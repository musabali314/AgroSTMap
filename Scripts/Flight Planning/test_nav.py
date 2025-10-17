#!/usr/bin/env python3
import time, math, threading
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from apriltag_ros.msg import AprilTagDetectionArray
from pymavlink import mavutil

# ========================== USER SETTINGS ==========================
CONNECTION = 'udpin:127.0.0.1:14550'

TAKEOFF_ALT      = 3.0
TAKEOFF_TIMEOUT  = 30.0
STREAM_HZ        = 8.5

# Waypoints in GAZEBO coordinates (Xg, Yg) -> we auto-map to ENU
GAZEBO_WPS = [(0.0, 0.0), (0.0, 4.0), (0.0, 10.0)]

# AprilTag map positions in GAZEBO coords (Xg, Yg)
TAG_MAP_GAZEBO = {0:(0.0, 0.0), 2:(0.0, 4.0), 5:(0.0, 10.0)}

# Toggle tag blending (XY only). If False, it flies pure odom PD.
USE_TAG_CORR    = True

# Camera→ENU mapping for AprilTag (x_c, y_c) from /tag_detections
# From your axis tests: forward made y_c increase; left/right text was flipped → set s_x=-1, s_y=+1
CAMERA_TO_ENU_SIGN = np.array([-1.0, +1.0], float)  # [E_off, N_off] = [ -x_c , +y_c ]
# ===================================================================

# ========================== TUNING ==========================
# Smooth & conservative
K_P_POS   = 0.25    # m/s per m
K_D_VEL   = 1.00    # m/s per (m/s)
MAX_VXY   = 0.30    # m/s
MAX_VD    = 0.25    # m/s (Down command magnitude)

ARRIVE_RADIUS = 0.50
ARRIVE_VEL    = 0.12
DWELL_NEXT    = 0.8
DWELL_FINAL   = 1.0
PRINT_PERIOD  = 0.6

# Tag fusion gates
TAG_STABLE_N  = 2         # frames in a row
TAG_GATE_DIST = 2.0       # max ENU dist between (odom) and tag-based absolute (m)
CORR_ALPHA    = 0.5       # blending factor to avoid “yanks”
# ===========================================================

def clamp(v, lo, hi): return max(lo, min(hi, v))

def gazebo_to_enu(Xg, Yg):
    """Your mapping: Gazebo +X → ENU North, Gazebo +Y → − ENU East"""
    E = -Yg
    N =  Xg
    return np.array([E, N], float)

def enu_to_gazebo(E, N):
    Xg = N
    Yg = -E
    return np.array([Xg, Yg], float)

def send_vel_local_ned(m, vN, vE, vD):
    TYPE_MASK = (
        (1<<0)|(1<<1)|(1<<2)|    # ignore pos
        (0<<3)|(0<<4)|(0<<5)|    # use vel
        (1<<6)|(1<<7)|(1<<8)|    # ignore accel
        (1<<9)|(1<<10)|(1<<11)   # ignore yaw/yawrate
    )
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        TYPE_MASK,
        0,0,0,  vN, vE, vD,  0,0,0,  0, 0
    )

class StateBus:
    """Reads /mavros/local_position/odom (ENU: x=E, y=N, z=Up)."""
    def __init__(self):
        self.lock = threading.Lock()
        self.xy  = np.zeros(2)  # ENU [E,N]
        self.vel = np.zeros(2)  # ENU [E,N]
        self.alt = 0.0          # Up
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self.cb, queue_size=30)
    def cb(self, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        with self.lock:
            self.xy  = np.array([p.x, p.y], float)
            self.vel = np.array([v.x, v.y], float)
            self.alt = float(p.z)
    def get(self):
        with self.lock:
            return self.xy.copy(), self.vel.copy(), float(self.alt)

class TagBus:
    def __init__(self):
        self.lock = threading.Lock()
        self.last = {}   # id -> (x_c, y_c, z_c)
        self.stk  = {}   # id -> streak
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.cb, queue_size=20)
    def cb(self, msg: AprilTagDetectionArray):
        closest = {}
        for det in msg.detections:
            if not det.id: continue
            tid = det.id[0]
            p = det.pose.pose.pose.position
            d = math.hypot(p.x, p.y)
            if tid not in closest or d < closest[tid][1]:
                closest[tid] = ((p.x, p.y, p.z), d)
        seen = set(closest.keys())
        with self.lock:
            for tid,(xyz,_) in closest.items():
                self.last[tid] = xyz
                self.stk[tid]  = self.stk.get(tid,0) + 1
            for tid in list(self.stk.keys()):
                if tid not in seen: self.stk[tid] = 0
    def get(self, tid):
        with self.lock:
            return self.last.get(tid, None), self.stk.get(tid, 0)

class Mission:
    def __init__(self, mav):
        self.mav = mav
        rospy.init_node("nav_gps_tagblend", anonymous=False)
        self.state = StateBus()
        self.tags  = TagBus()
        self.corr_xy = np.zeros(2)
        self._last_print = 0.0

        # Pre-compute ENU waypoints from Gazebo list
        self.WP_ENU = [gazebo_to_enu(*wp) for wp in GAZEBO_WPS]

    # ---------- FCU ----------
    def arm_and_takeoff(self):
        print("[MISSION] Arming...")
        self.mav.set_mode("GUIDED"); time.sleep(0.2)
        self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component,
                                       mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                       0, 1,0,0,0,0,0,0)
        ack = self.mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if not (ack and ack.result==mavutil.mavlink.MAV_RESULT_ACCEPTED):
            print("[ERR] Arming failed"); return False

        print(f"[MISSION] Takeoff to {TAKEOFF_ALT:.1f} m...")
        self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component,
                                       mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                       0,0,0,0,0,0,0,TAKEOFF_ALT)
        t0=time.time()
        while time.time()-t0 < TAKEOFF_TIMEOUT and not rospy.is_shutdown():
            _,_,alt = self.state.get()
            if alt >= TAKEOFF_ALT-0.2:
                print("[OK ] Takeoff complete")
                return True
            time.sleep(0.05)
        print("[ERR] Takeoff timeout"); return False

    # ---------- Control helpers ----------
    def pd_vel_cmd(self, err_enu, vel_enu):
        v_enu = K_P_POS*err_enu - K_D_VEL*vel_enu
        spd = float(np.linalg.norm(v_enu))
        if spd > MAX_VXY:
            v_enu *= (MAX_VXY / spd)
        # ENU -> NED for LOCAL_NED
        vN = float(v_enu[1])         # North = ENU.y
        vE = float(v_enu[0])         # East  = ENU.x
        return vN, vE

    # ---------- Tag fusion ----------
    def try_tag_fusion(self, xy_raw):
        """Blend toward absolute ENU from the nearest visible tag. Ignores tag Z."""
        if not USE_TAG_CORR:
            return False

        # choose the tag with best streak among those we know
        picked = None
        best_streak = 0
        for tid in TAG_MAP_GAZEBO.keys():
            cam, streak = self.tags.get(tid)
            if cam and streak >= TAG_STABLE_N and streak >= best_streak:
                picked = (tid, cam, streak)
                best_streak = streak
        if not picked:
            return False

        tid, cam, streak = picked
        x_c, y_c, z_c = cam

        # known tag position in ENU
        tag_enu = gazebo_to_enu(*TAG_MAP_GAZEBO[tid])  # (E,N)

        # camera offsets -> ENU (XY only), using the sign matrix you confirmed
        enu_off = CAMERA_TO_ENU_SIGN * np.array([x_c, y_c], float)
        xy_from_tag = tag_enu + enu_off

        # sanity gate: only fuse if odom is reasonably near the tag estimate
        if np.linalg.norm(xy_from_tag - xy_raw) > TAG_GATE_DIST:
            return False

        # blend correction
        delta = xy_from_tag - xy_raw
        self.corr_xy = (1.0 - CORR_ALPHA)*self.corr_xy + CORR_ALPHA*delta

        # pretty print in Gazebo axes so you can eyeball it in sim
        # current fused estimate (for text only)
        xy_fused = xy_raw + self.corr_xy
        Xg_t, Yg_t = TAG_MAP_GAZEBO[tid]
        Xg_d, Yg_d = enu_to_gazebo(*xy_fused)
        dXg = Xg_d - Xg_t   # +RIGHT, -LEFT
        dYg = Yg_d - Yg_t   # +AHEAD(+Y), -BEHIND(-Y)
        side = "RIGHT" if dXg > 0 else "LEFT"
        fore = "AHEAD(+Y)" if dYg > 0 else "BEHIND(-Y)"
        print(f"[REL] tag {tid}: Gazebo Δ(X:{dXg:+.2f}, Y:{dYg:+.2f}) → {side} {abs(dXg):.2f} m, {fore} {abs(dYg):.2f} m")
        print(f"[FUSE] tag {tid}: corr -> {self.corr_xy.round(3)} toward ENU {tuple(xy_from_tag.round(2))}")
        return True

    # ---------- Mission ----------
    def run(self):
        if not self.arm_and_takeoff(): return
        rate = rospy.Rate(STREAM_HZ)

        wp_idx = 0
        print(f"[GO ] Pursuing WP{wp_idx} (Gazebo {GAZEBO_WPS[wp_idx]})")

        while not rospy.is_shutdown():
            xy_raw, vel_enu, alt = self.state.get()
            fused = self.try_tag_fusion(xy_raw)
            xy = xy_raw + self.corr_xy

            tgt = self.WP_ENU[wp_idx]
            err = tgt - xy
            dist = float(np.linalg.norm(err))
            gspd = float(np.linalg.norm(vel_enu))

            # vertical hold @ TAKEOFF_ALT: (Up error) → Down velocity
            vD = clamp(TAKEOFF_ALT - alt, -MAX_VD, MAX_VD) * (-1.0)

            now = time.time()
            if now - self._last_print > PRINT_PERIOD:
                Xg, Yg = enu_to_gazebo(*xy)
                Xg_t, Yg_t = GAZEBO_WPS[wp_idx]
                print(f"[WP ] {wp_idx}/{len(self.WP_ENU)-1} | pos_ENU=({xy[0]:+.2f},{xy[1]:+.2f}) "
                      f"pos_GZ=({Xg:+.2f},{Yg:+.2f}) tgt_GZ=({Xg_t:.2f},{Yg_t:.2f}) "
                      f"dist={dist:.2f} gspd={gspd:.2f} alt={alt:.2f}"
                      + ("" if not fused else "  [corr]"))
                self._last_print = now

            # arrival
            if dist <= ARRIVE_RADIUS and gspd <= ARRIVE_VEL:
                print(f"[OK ] Arrived WP{wp_idx} @ Gazebo {GAZEBO_WPS[wp_idx]}")
                dwell = DWELL_FINAL if wp_idx==len(self.WP_ENU)-1 else DWELL_NEXT
                t0 = time.time()
                while time.time()-t0 < dwell and not rospy.is_shutdown():
                    # hold with small PD to keep it steady at the waypoint
                    xy_hold, vel_hold, alt2 = self.state.get()
                    err_hold = tgt - (xy_hold + self.corr_xy)
                    vN, vE = self.pd_vel_cmd(err_hold, vel_hold)
                    vD2 = clamp(TAKEOFF_ALT - alt2, -MAX_VD, MAX_VD) * (-1.0)
                    send_vel_local_ned(self.mav, vN, vE, vD2)
                    rate.sleep()

                if wp_idx == len(self.WP_ENU)-1:
                    print("[LAND] Final WP reached; LAND.")
                    self.mav.set_mode("LAND")
                    return
                wp_idx += 1
                print(f"[GO ] Pursuing WP{wp_idx} (Gazebo {GAZEBO_WPS[wp_idx]})")
                continue

            vN, vE = self.pd_vel_cmd(err, vel_enu)
            send_vel_local_ned(self.mav, vN, vE, vD)
            rate.sleep()

# ---------- entry ----------
def wait_hb(m):
    print("[SYS] Waiting for heartbeat..."); m.wait_heartbeat()
    print(f"[SYS] Heartbeat from system {m.target_system} component {m.target_component}")

def main():
    m = mavutil.mavlink_connection(CONNECTION)
    wait_hb(m)
    nav = Mission(m)
    try:
        nav.run()
    finally:
        try: m.close()
        except: pass

if __name__=="__main__":
    main()
