#!/usr/bin/env python3
import math, time, sys, signal, argparse
from datetime import datetime
from pymavlink import mavutil

# ------------------------------------------------------------
# Utility
# ------------------------------------------------------------
def now_str():
    return datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%S.%fZ")

def is_serial(conn): return conn.startswith("/dev/")

def request_message_interval(master, msg_id, hz):
    if hz <= 0: return
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0, msg_id, int(1e6/hz), 0,0,0,0,0
    )

# ------------------------------------------------------------
# FCU setup
# ------------------------------------------------------------
def connect_to_drone(conn, baud=115200):
    print(f"[{now_str()}] Connecting to {conn} ...")
    master = mavutil.mavlink_connection(conn, baud=baud) if is_serial(conn) else mavutil.mavlink_connection(conn)
    master.wait_heartbeat()
    print(f"[{now_str()}] Heartbeat from system {master.target_system} component {master.target_component}")
    return master

def set_mode(master, mode):
    mapping = master.mode_mapping()
    if mode not in mapping:
        print(f"[{now_str()}] Mode '{mode}' not available.")
        return False
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mapping[mode]
    )
    time.sleep(1.0)
    print(f"[{now_str()}] Mode set to {mode}.")
    return True

def arm(master, arm_it=True):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1 if arm_it else 0, 0,0,0,0,0,0)
    print(f"[{now_str()}] {'Arming' if arm_it else 'Disarming'} command sent.")
    time.sleep(4.0)

def takeoff(master, alt):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,0,0,0,0,0,0,alt)
    print(f"[{now_str()}] Takeoff to {alt:.2f} m commanded.")
    time.sleep(5.0)

# ------------------------------------------------------------
# Telemetry
# ------------------------------------------------------------
def get_local_position(master):
    msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=True)
    return msg.x, msg.y, msg.z   # N, E, D (D positive down)

def get_yaw(master):
    msg = master.recv_match(type="ATTITUDE", blocking=True)
    return msg.yaw  # radians

# ------------------------------------------------------------
# Calibration (origin + yaw)
# ------------------------------------------------------------
def average_origin(master, duration_s=10.0):
    print(f"[{now_str()}] Averaging position & yaw for {duration_s}s to set local origin...")
    n_sum=e_sum=d_sum=yaw_sum=0.0; count=0
    start=time.time()
    while time.time()-start < duration_s:
        n,e,d = get_local_position(master)
        yaw = get_yaw(master)
        n_sum+=n; e_sum+=e; d_sum+=d; yaw_sum+=yaw; count+=1
        time.sleep(0.1)
    n0,e0,d0,yaw0 = n_sum/count, e_sum/count, d_sum/count, yaw_sum/count
    print(f"[{now_str()}] Origin set to NED ({n0:+.2f},{e0:+.2f},{d0:+.2f}), yaw0={math.degrees(yaw0):+.1f}°")
    return n0,e0,d0,yaw0

# ------------------------------------------------------------
# Coordinate transforms
# ------------------------------------------------------------
def body_to_world(n_body, e_body, yaw0):
    c, s = math.cos(yaw0), math.sin(yaw0)
    return c*n_body - s*e_body, s*n_body + c*e_body

def rotate_by_yaw(n,e,yaw0):
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    return c*n - s*e, s*n + c*e

# ------------------------------------------------------------
# Velocity controller (to position targets)
# ------------------------------------------------------------
def send_local_velocity_ned(master, vn, ve, vd, yaw_target):
    """Send a velocity setpoint (NED) with yaw lock."""
    mask = (1<<0)|(1<<1)|(1<<2)|(1<<6)|(1<<7)|(1<<8)|(1<<11)
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
        0,0,0, vn,ve,vd, 0,0,0, yaw_target,0.0
    )

def clamp(x, lo, hi): return lo if x<lo else hi if x>hi else x

def glide_to_target(master, n_cmd, e_cmd, d_cmd, yaw_lock,
                    v_xy_max=0.6, v_z_max=0.6,
                    pos_tol=0.8, timeout_s=40):
    """Move toward a WORLD target at capped velocity until within tolerance."""
    t0 = time.time()
    while True:
        n,e,d = get_local_position(master)
        en,ee,ed = n_cmd-n, e_cmd-e, d_cmd-d
        dist_xy = math.hypot(en,ee)
        dist_3d = math.sqrt(en*en + ee*ee + ed*ed)

        # Velocity command
        if dist_xy > 1e-3:
            vn = (en/dist_xy)*min(v_xy_max, dist_xy)
            ve = (ee/dist_xy)*min(v_xy_max, dist_xy)
        else:
            vn=ve=0.0
        vd = clamp(0.8*ed, -v_z_max, v_z_max)

        send_local_velocity_ned(master, vn, ve, vd, yaw_lock)

        alt = -d
        sys.stdout.write(f"\r[{now_str()}] To target | XY={dist_xy:.2f} Z={abs(ed):.2f} | Alt={alt:+.2f} | v=({vn:+.2f},{ve:+.2f},{vd:+.2f})   ")
        sys.stdout.flush()

        if dist_3d <= pos_tol:
            print(f"\n[{now_str()}] Target reached within {pos_tol:.2f} m.")
            # small brake
            for _ in range(5):
                send_local_velocity_ned(master, 0,0,0, yaw_lock)
                time.sleep(0.05)
            return True

        if time.time()-t0 > timeout_s:
            print(f"\n[{now_str()}] Timeout before reaching target.")
            for _ in range(5):
                send_local_velocity_ned(master, 0,0,0, yaw_lock)
                time.sleep(0.05)
            return False

        time.sleep(0.05)

# ------------------------------------------------------------
# Mission
# ------------------------------------------------------------
def do_custom_path(master, alt_m, n0,e0,d0,yaw0):
    d_target = -alt_m
    yaw_lock = yaw0

    waypoints_body = [
        (20.0, 0.0, d_target, "Leg 1: forward (15,0)"),
        (20.0, 2.0, d_target, "Leg 2: right (15,2)"),
        (0.0,  2.0, d_target, "Leg 3: back (0,2)"),
        (0.0,  4.0, d_target, "Leg 4: right (0,4)"),
        (20.0, 4.0, d_target, "Leg 5: forward (15,4)")
    ]

    for i,(n_b,e_b,d_b,desc) in enumerate(waypoints_body,1):
        n_w,e_w = body_to_world(n_b,e_b,yaw0)
        n_cmd,e_cmd,d_cmd = n0+n_w, e0+e_w, d0+d_b
        print(f"\n[{now_str()}] --- {desc} ({i}/{len(waypoints_body)}) ---")
        glide_to_target(master, n_cmd,e_cmd,d_cmd,yaw_lock,
                        v_xy_max=0.6, v_z_max=0.6, pos_tol=0.8, timeout_s=40)
        time.sleep(1.0)

    print(f"[{now_str()}] Descending to 0.1 m AGL ...")
    glide_to_target(master, n_cmd,e_cmd,d0-0.1,yaw_lock,
                    v_xy_max=0.6,v_z_max=0.2,pos_tol=0.2,timeout_s=20)

    print(f"[{now_str()}] LAND command sent.")
    master.mav.set_mode_send(master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        master.mode_mapping().get("LAND",0))
    time.sleep(2.0)

# ------------------------------------------------------------
# Main
# ------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Yaw-locked velocity-glide autonomous path.")
    ap.add_argument("--conn", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--alt", type=float, default=5.0)
    ap.add_argument("--mode", default="GUIDED")
    args = ap.parse_args()

    def handle(sig,frame):
        print(f"\n[{now_str()}] Ctrl-C → switching to LAND for safety.")
        try:
            master.mav.set_mode_send(master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                master.mode_mapping().get("LAND",0))
        except: pass
        sys.exit(0)
    signal.signal(signal.SIGINT, handle)

    master = connect_to_drone(args.conn, baud=args.baud)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10)

    n0,e0,d0,yaw0 = average_origin(master, duration_s=10)
    set_mode(master, args.mode)
    arm(master, True)
    takeoff(master, args.alt)

    do_custom_path(master, args.alt, n0,e0,d0,yaw0)
    print(f"[{now_str()}] Mission complete.")

if __name__ == "__main__":
    sys.exit(main())
