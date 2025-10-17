#!/usr/bin/env python3
import time, math, threading
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from apriltag_ros.msg import AprilTagDetectionArray
from pymavlink import mavutil

# ==================== USER KNOBS ====================
CONN_STR        = 'udpin:127.0.0.1:14550'
TAKEOFF_ALT_M   = 6.0
TAKEOFF_TIMEOUT = 40.0
STREAM_HZ       = 8.5

# burst settings (>= 3s as requested)
BURST_SEC       = 2.0
COAST_SEC       = 1.0

# local-NED burst speeds
V_N_MAG         = 0.7      # m/s  (+N / -N)
V_E_MAG         = 0.7      # m/s  (+E / -E)
V_D_MAG         = 0.4      # m/s  (+Down / -Down)

# body-NED burst speeds
V_FWD_MAG       = 0.7      # m/s  (+forward / -forward)
V_RIGHT_MAG     = 1.5      # m/s  (+right / -right)
YAW_RATE_DPS    = 25.0     # deg/s for yaw bursts

# camera test
DO_CAMERA_TEST  = True     # set False to skip
TAG_STREAK_N    = 2        # require at least N consecutive frames
PRINT_EVERY_S   = 0.5
# ====================================================

def clamp(v, lo, hi): return max(lo, min(hi, v))

# ---------- MAVLink helpers ----------
def wait_heartbeat(m):
    print("[SYS] Waiting for heartbeat...")
    m.wait_heartbeat()
    print(f"[SYS] Heartbeat from system {m.target_system} component {m.target_component}")

def set_mode(m, mode_str):
    m.set_mode(mode_str)

def arm_and_takeoff(m):
    print(f"[MISSION] Arming & takeoff to {TAKEOFF_ALT_M:.1f} m...")
    set_mode(m, "GUIDED")
    time.sleep(0.3)
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                            0, 1,0,0,0,0,0,0)
    ack = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if not (ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED):
        raise RuntimeError("Arming rejected")

    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                            0,0,0,0,0,0,0,TAKEOFF_ALT_M)

    t0 = time.time()
    while time.time() - t0 < TAKEOFF_TIMEOUT and not rospy.is_shutdown():
        if state.alt >= TAKEOFF_ALT_M - 0.2:
            print("[OK ] Takeoff complete")
            return
        time.sleep(0.05)
    raise RuntimeError("Takeoff timeout")

def send_vel_local_ned(m, vN, vE, vD):
    TYPE_MASK = (
        (1<<0)|(1<<1)|(1<<2) |    # ignore x,y,z pos
        (0<<3)|(0<<4)|(0<<5) |    # use vx,vy,vz
        (1<<6)|(1<<7)|(1<<8) |    # ignore ax,ay,az
        (1<<9)|(1<<10)            # ignore yaw,yaw-rate
    )
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        TYPE_MASK,
        0,0,0, vN, vE, vD, 0,0,0, 0, 0
    )

def send_vel_body_ned(m, vx_fwd, vy_right, vz_down, yaw_rate_dps=0.0):
    TYPE_MASK = (
        (1<<0)|(1<<1)|(1<<2) |    # ignore pos
        (0<<3)|(0<<4)|(0<<5) |    # use vel
        (1<<6)|(1<<7)|(1<<8) |    # ignore accel
        (1<<9)|(0<<10)            # use yaw rate only
    )
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        TYPE_MASK,
        0,0,0, vx_fwd, vy_right, vz_down, 0,0,0, 0, math.radians(yaw_rate_dps)
    )

def hover_local(m): send_vel_local_ned(m, 0,0,0)
def hover_body(m):  send_vel_body_ned(m, 0,0,0, 0)

# ---------- State buses ----------
class OdomBus:
    def __init__(self):
        self.lock = threading.Lock()
        self.E = 0.0; self.N = 0.0; self.Up = 0.0
        self.vE = 0.0; self.vN = 0.0; self.vU = 0.0
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self.cb, queue_size=30)
    def cb(self, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        with self.lock:
            self.E, self.N, self.Up = float(p.x), float(p.y), float(p.z)
            self.vE, self.vN, self.vU = float(v.x), float(v.y), float(v.z)
    @property
    def alt(self):
        with self.lock: return self.Up
    def get(self):
        with self.lock:
            return (self.E, self.N, self.Up, self.vE, self.vN, self.vU)

class TagBus:
    def __init__(self):
        self.lock = threading.Lock()
        self.streak = {}  # id->count
        self.last   = {}  # id->(x_c,y_c,z_c)
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.cb, queue_size=10)
        self._last_print = 0.0
    def cb(self, msg: AprilTagDetectionArray):
        now = time.time()
        # choose closest per id in image plane
        closest = {}
        for d in msg.detections:
            if not d.id: continue
            tid = d.id[0]
            p = d.pose.pose.pose.position
            dxy = math.hypot(p.x, p.y)
            if tid not in closest or dxy < closest[tid][1]:
                closest[tid] = ((p.x, p.y, p.z), dxy)
        with self.lock:
            seen = set()
            for tid,(xyz,_) in closest.items():
                self.last[tid] = xyz
                self.streak[tid] = self.streak.get(tid,0) + 1
                seen.add(tid)
            for tid in list(self.streak.keys()):
                if tid not in seen: self.streak[tid] = 0
        if now - self._last_print > PRINT_EVERY_S and closest:
            ids = sorted([k for k in closest.keys()])
            print(f"[TAG] seen: {ids}")
            self._last_print = now
    def get(self, tid):
        with self.lock:
            return self.last.get(tid, None), self.streak.get(tid, 0)
    def any(self):
        with self.lock:
            if not self.last: return None
            # return id with highest streak
            best = max(self.streak.items(), key=lambda kv: kv[1])[0]
            return best, self.last[best], self.streak[best]

# ---------- Measurement helpers ----------
def measure_delta(fn_apply, sec, sample_rate_hz=STREAM_HZ):
    """Run fn_apply() continuously for `sec`, return ΔE,ΔN,ΔUp and Δyaw (deg) from ODOM/ATTITUDE."""
    E0,N0,U0,_,_,_ = state.get()
    t0 = time.time()
    while time.time() - t0 < sec and not rospy.is_shutdown():
        fn_apply()
        time.sleep(1.0/sample_rate_hz)
    E1,N1,U1,_,_,_ = state.get()
    return (E1-E0, N1-N0, U1-U0)

# ---------- Run bursts & summarize ----------
def run_local_bursts(m):
    print("[INFO] LOCAL_NED bursts (expect ENU on odom).")
    results = {}

    # +N
    dE,dN,dU = measure_delta(lambda: send_vel_local_ned(m, +V_N_MAG, 0.0, 0.0), BURST_SEC)
    hover_local(m); time.sleep(COAST_SEC)
    print(f"[BURST] NORTH + | ΔE={dE:+.2f} ΔN={dN:+.2f} ΔUp={dU:+.2f}")
    results[("vN","+")] = (dE,dN,dU)

    # -N
    dE,dN,dU = measure_delta(lambda: send_vel_local_ned(m, -V_N_MAG, 0.0, 0.0), BURST_SEC)
    hover_local(m); time.sleep(COAST_SEC)
    print(f"[BURST] NORTH - | ΔE={dE:+.2f} ΔN={dN:+.2f} ΔUp={dU:+.2f}")
    results[("vN","-")] = (dE,dN,dU)

    # +E
    dE,dN,dU = measure_delta(lambda: send_vel_local_ned(m, 0.0, +V_E_MAG, 0.0), BURST_SEC)
    hover_local(m); time.sleep(COAST_SEC)
    print(f"[BURST] EAST  + | ΔE={dE:+.2f} ΔN={dN:+.2f} ΔUp={dU:+.2f}")
    results[("vE","+")] = (dE,dN,dU)

    # -E
    dE,dN,dU = measure_delta(lambda: send_vel_local_ned(m, 0.0, -V_E_MAG, 0.0), BURST_SEC)
    hover_local(m); time.sleep(COAST_SEC)
    print(f"[BURST] EAST  - | ΔE={dE:+.2f} ΔN={dN:+.2f} ΔUp={dU:+.2f}")
    results[("vE","-")] = (dE,dN,dU)

    # # +Down
    # dE,dN,dU = measure_delta(lambda: send_vel_local_ned(m, 0.0, 0.0, +V_D_MAG), BURST_SEC)
    # hover_local(m); time.sleep(COAST_SEC)
    # print(f"[BURST] DOWN  + | ΔE={dE:+.2f} ΔN={dN:+.2f} ΔUp={dU:+.2f}")
    # results[("vD","+")] = (dE,dN,dU)

    # # -Down (i.e., Up)
    # dE,dN,dU = measure_delta(lambda: send_vel_local_ned(m, 0.0, 0.0, -V_D_MAG), BURST_SEC)
    # hover_local(m); time.sleep(COAST_SEC)
    # print(f"[BURST] DOWN  - | ΔE={dE:+.2f} ΔN={dN:+.2f} ΔUp={dU:+.2f}")
    # results[("vD","-")] = (dE,dN,dU)

    return results

def run_body_bursts(m):
    print("[INFO] BODY_NED bursts (vx=+forward, vy=+right, vz=+down).")
    results = {}

    # +forward
    dE,dN,dU = measure_delta(lambda: send_vel_body_ned(m, +V_FWD_MAG, 0.0, 0.0, 0.0), BURST_SEC)
    hover_body(m); time.sleep(COAST_SEC)
    print(f"[BURST] BODY FWD + | ΔE={dE:+.2f} ΔN={dN:+.2f} ΔUp={dU:+.2f}")
    results[("vx","+")] = (dE,dN,dU)

    # -forward
    dE,dN,dU = measure_delta(lambda: send_vel_body_ned(m, -V_FWD_MAG, 0.0, 0.0, 0.0), BURST_SEC)
    hover_body(m); time.sleep(COAST_SEC)
    print(f"[BURST] BODY FWD - | ΔE={dE:+.2f} ΔN={dN:+.2f} ΔUp={dU:+.2f}")
    results[("vx","-")] = (dE,dN,dU)

    # +right
    dE,dN,dU = measure_delta(lambda: send_vel_body_ned(m, 0.0, +V_RIGHT_MAG, 0.0, 0.0), BURST_SEC)
    hover_body(m); time.sleep(COAST_SEC)
    print(f"[BURST] BODY RIGHT + | ΔE={dE:+.2f} ΔN={dN:+.2f} ΔUp={dU:+.2f}")
    results[("vy","+")] = (dE,dN,dU)

    # -right
    dE,dN,dU = measure_delta(lambda: send_vel_body_ned(m, 0.0, -V_RIGHT_MAG, 0.0, 0.0), BURST_SEC)
    hover_body(m); time.sleep(COAST_SEC)
    print(f"[BURST] BODY RIGHT - | ΔE={dE:+.2f} ΔN={dN:+.2f} ΔUp={dU:+.2f}")
    results[("vy","-")] = (dE,dN,dU)

    # yaw + (CCW positive in MAVLink)
    dE0,dN0,_ = state.get()[:3]
    _ = measure_delta(lambda: send_vel_body_ned(m, 0.0, 0.0, 0.0, +YAW_RATE_DPS), BURST_SEC)
    hover_body(m); time.sleep(COAST_SEC)
    dE1,dN1,_ = state.get()[:3]
    dyaw_rel = "CCW(+)"  # in ArduPilot/MAVLink, +yaw_rate is CCW
    print(f"[BURST] BODY YAW  + | Δpos≈({(dE1-dE0):+.2f},{(dN1-dN0):+.2f}) (rot in place {dyaw_rel})")
    results[("yaw","+")] = None

    # yaw -
    dE0,dN0,_ = state.get()[:3]
    _ = measure_delta(lambda: send_vel_body_ned(m, 0.0, 0.0, 0.0, -YAW_RATE_DPS), BURST_SEC)
    hover_body(m); time.sleep(COAST_SEC)
    dE1,dN1,_ = state.get()[:3]
    print(f"[BURST] BODY YAW  - | Δpos≈({(dE1-dE0):+.2f},{(dN1-dN0):+.2f}) (rot in place CW(-))")
    results[("yaw","-")] = None

    return results

def collect_camera_response(m, tagbus: TagBus):
    if not DO_CAMERA_TEST:
        print("[SKIP] Camera/Tag test is disabled.")
        return None

    # choose the most stable visible tag
    picked = None
    wait_t0 = time.time()
    while time.time() - wait_t0 < 8.0 and not rospy.is_shutdown():
        q = tagbus.any()
        if q:
            tid, xyz, streak = q
            if streak >= TAG_STREAK_N:
                picked = tid
                break
        time.sleep(0.05)

    if picked is None:
        print("[WARN] No tag visible with sufficient streak; skipping camera test.")
        return None

    def get_xyc():
        xyz, stk = tagbus.get(picked)
        return (None,0) if xyz is None else ((xyz[0], xyz[1]), stk)

    print(f"[INFO] Camera/Tag bursts using tag id={picked} (need ~3s steady per axis).")

    def delta_cam(fn_apply):
        # average start
        xs, ys = [], []
        t0 = time.time()
        while time.time() - t0 < 0.6:
            xy, s = get_xyc()
            if xy: xs.append(xy[0]); ys.append(xy[1])
            fn_apply()
            time.sleep(1.0/STREAM_HZ)
        x0 = np.mean(xs) if xs else 0.0
        y0 = np.mean(ys) if ys else 0.0

        # run
        t1 = time.time()
        while time.time() - t1 < BURST_SEC:
            fn_apply()
            time.sleep(1.0/STREAM_HZ)

        # average end
        xe, ye = [], []
        t2 = time.time()
        while time.time() - t2 < 0.6:
            xy, s = get_xyc()
            if xy: xe.append(xy[0]); ye.append(xy[1])
            hover_body(m); time.sleep(1.0/STREAM_HZ)
        x1 = np.mean(xe) if xe else x0
        y1 = np.mean(ye) if ye else y0
        return (x1-x0, y1-y0)

    # BODY forward +
    dx1, dy1 = delta_cam(lambda: send_vel_body_ned(m, +V_FWD_MAG, 0.0, 0.0, 0.0))
    time.sleep(COAST_SEC)
    print(f"[CAM ] BODY FWD +  -> Δ(x_c,y_c)=({dx1:+.3f},{dy1:+.3f})")

    # BODY right +
    dx2, dy2 = delta_cam(lambda: send_vel_body_ned(m, 0.0, +V_RIGHT_MAG, 0.0, 0.0))
    time.sleep(COAST_SEC)
    print(f"[CAM ] BODY RIGHT +-> Δ(x_c,y_c)=({dx2:+.3f},{dy2:+.3f})")

    # inference (signs only)
    inf = {
        "FWD+ increases": {
            "x_c": dx1 > 0.0,
            "y_c": dy1 > 0.0
        },
        "RIGHT+ increases": {
            "x_c": dx2 > 0.0,
            "y_c": dy2 > 0.0
        },
        "raw": {"FWD+": (dx1,dy1), "RIGHT+": (dx2,dy2)}
    }
    return inf

def summarize(local_res, body_res, cam_inf):
    print("\n=================== SUMMARY ===================")
    print("MAVROS /mavros/local_position/odom is ENU (E,N,Up).")
    # LOCAL mapping
    ln = local_res[("vN","+")]
    le = local_res[("vE","+")]
    ld = local_res[("vD","+")]
    print("\n[LOCAL_NED command → ODOM (ENU) delta]")
    print(f" vN +  ⇒ ΔN≈{ln[1]:+.2f} (should be +), ΔE≈{ln[0]:+.2f}")
    print(f" vE +  ⇒ ΔE≈{le[0]:+.2f} (should be +), ΔN≈{le[1]:+.2f}")
    print(f" vD +  ⇒ ΔUp≈{ld[2]:+.2f} (should be − because Down+)")

    # Implied Gazebo mapping from your previous observation
    # (not measured directly here, but stated explicitly)
    print("\n[Gazebo ↔ ENU heuristic based on observed sims]")
    print(" Gazebo +X ≈ ENU North (N+)")
    print(" Gazebo +Y ≈ − ENU East (−E+)")
    print(" Gazebo +Z ≈ ENU Up (+)")

    # BODY mapping (at current yaw)
    vxp = body_res[("vx","+")]
    vyp = body_res[("vy","+")]
    print("\n[BODY_NED command → ODOM (ENU) delta (at test yaw)]")
    print(f" FWD+  ⇒ ΔE≈{vxp[0]:+.2f}, ΔN≈{vxp[1]:+.2f}, ΔUp≈{vxp[2]:+.2f}")
    print(f" RIGHT+⇒ ΔE≈{vyp[0]:+.2f}, ΔN≈{vyp[1]:+.2f}, ΔUp≈{vyp[2]:+.2f}")
    print(" Yaw+: CCW (+), Yaw-: CW (−)\n")

    if cam_inf:
        dx1,dy1 = cam_inf["raw"]["FWD+"]
        dx2,dy2 = cam_inf["raw"]["RIGHT+"]
        print("[Camera/AprilTag response (tag frame)]")
        print(f" BODY FWD +   → Δ(x_c,y_c)=({dx1:+.3f},{dy1:+.3f})")
        print(f" BODY RIGHT + → Δ(x_c,y_c)=({dx2:+.3f},{dy2:+.3f})")

        # Guidance on sign-correct fusion
        # Camera frame (AprilTag ROS default): x to RIGHT (image), y DOWN, z forward (optical).
        # With a DOWNWARD camera, your motion +FWD/+RIGHT will drive (x_c,y_c) with these measured signs.
        print("\n[Use this to set tag→ENU correction signs]")
        fx = "increase" if dx1>0 else "decrease"
        fy = "increase" if dy1>0 else "decrease"
        rx = "increase" if dx2>0 else "decrease"
        ry = "increase" if dy2>0 else "decrease"
        print(f" When you move BODY FWD(+), x_c will {fx}, y_c will {fy}.")
        print(f" When you move BODY RIGHT(+), x_c will {rx}, y_c will {ry}.")

        print("\n Rule of thumb for fusion (XY only):")
        print("  If your goal is to be OVER the tag, drive body so that x_c→0 and y_c→0 with the measured signs.")
        print("  For ENU blending, map (x_c,y_c) into ENU using the sign matrix you just observed,")
        print("  then apply a small blended correction toward your known tag ENU position.")
    print("================================================\n")

# ---------- Main ----------
class NodeState:
    def __init__(self):
        self._odom = OdomBus()
        self._tags = TagBus()
    @property
    def alt(self): return self._odom.alt
    def get(self): return self._odom.get()
    def tagbus(self): return self._tags

state: OdomBus
tags: TagBus

def main():
    global state, tags
    rospy.init_node("axis_calibrator", anonymous=False)
    state = NodeState()._odom
    tags  = NodeState()._tags

    m = mavutil.mavlink_connection(CONN_STR)
    wait_heartbeat(m)
    arm_and_takeoff(m)

    # keep altitude steady during tests (simple hold loop in background)
    stop_evt = threading.Event()
    def alt_hold_loop():
        while not stop_evt.is_set() and not rospy.is_shutdown():
            _,_,up,_,_,_ = state.get()
            vD = clamp(TAKEOFF_ALT_M - up, -0.3, 0.3) * (-1.0)  # positive down when above target
            # nudge tiny down/up to stay around target during idle
            send_vel_local_ned(m, 0.0, 0.0, vD)
            time.sleep(0.05)
    th = threading.Thread(target=alt_hold_loop, daemon=True)
    th.start()

    try:
        local_res = run_local_bursts(m)
        body_res  = run_body_bursts(m)
        cam_inf   = collect_camera_response(m, tags)
        summarize(local_res, body_res, cam_inf)
    finally:
        stop_evt.set()
        hover_local(m); time.sleep(0.5)
        print("[PHASE] Landing...")
        m.set_mode("LAND")
        try: m.close()
        except: pass

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INT] User interrupted, exiting.")
