#!/usr/bin/env python3
import math, time, sys, signal, argparse
from datetime import datetime
from pymavlink import mavutil


# ------------------------------------------------------------
# Utility
# ------------------------------------------------------------

def now_str():
    return datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%S.%fZ")

def is_serial(conn):
    return conn.startswith("/dev/")

def request_message_interval(master, msg_id, hz):
    if hz <= 0:
        return
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0, msg_id, int(1e6 / hz), 0,0,0,0,0
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
    time.sleep(10.0)

def takeoff(master, alt):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,0,0,0,0,0,0,alt)
    print(f"[{now_str()}] Takeoff to {alt:.2f} m commanded.")
    time.sleep(5.0)


# ------------------------------------------------------------
# Telemetry helpers
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
    """Convert position in BODY frame to Earth frame using yaw0 (rad)."""
    c, s = math.cos(yaw0), math.sin(yaw0)
    n_world = c*n_body - s*e_body
    e_world = s*n_body + c*e_body
    return n_world, e_world

def rotate_by_yaw(n,e,yaw0):
    """Rotate N,E so that yaw0 becomes zero (for display)."""
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    return c*n - s*e, s*n + c*e


# ------------------------------------------------------------
# Guidance primitives
# ------------------------------------------------------------

def send_local_position_target(master, n, e, d):
    mask = 0b0000111111111000  # position-only control
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
        n,e,d, 0,0,0, 0,0,0, 0,0)

def wait_until_reached(master, n_tgt,e_tgt,d_tgt,n0,e0,d0,yaw0,
                       pos_tol=1.0, timeout_s=20):
    """Wait until drone is within pos_tol (in meters) of the target."""
    t0=time.time()
    while True:
        n_raw,e_raw,d_raw = get_local_position(master)
        n_rel,e_rel = n_raw-n0, e_raw-e0
        n_disp,e_disp = rotate_by_yaw(n_rel,e_rel,yaw0)
        d = d_raw-d0
        dist = math.sqrt((n_disp-n_tgt)**2 + (e_disp-e_tgt)**2 + (d-d_tgt)**2)
        altitude = -d  # altitude above origin
        sys.stdout.write(f"\r[{now_str()}] N:{n_disp:+.2f} E:{e_disp:+.2f} Alt:{altitude:+.2f} m | err {dist:.2f} m   ")
        sys.stdout.flush()
        if dist <= pos_tol:
            print(f"\n[{now_str()}] Waypoint reached (error {dist:.2f} m).")
            return True
        if time.time()-t0 > timeout_s:
            print(f"\n[{now_str()}] Timeout ({timeout_s}s) before reaching target.")
            return False
        time.sleep(0.1)


# ------------------------------------------------------------
# Mission
# ------------------------------------------------------------

def do_square(master, side_m, alt_m, n0,e0,d0,yaw0):
    d_target = -alt_m

    # Square waypoints defined in BODY frame
    waypoints_body = [
        (0.0,        0.0,        d_target, "Start"),
        (side_m,     0.0,        d_target, "Forward"),
        (side_m,     side_m,     d_target, "Right"),
        (0.0,        side_m,     d_target, "Back"),
        (0.0,        0.0,        d_target, "Return")
    ]

    for i,(n_b,e_b,d_t,vdir) in enumerate(waypoints_body,1):
        # Convert from body-frame to world-frame and add origin offset
        n_world, e_world = body_to_world(n_b, e_b, yaw0)
        n_cmd, e_cmd, d_cmd = n0 + n_world, e0 + e_world, d0 + d_t

        print(f"\n[{now_str()}] --- Leg {i}/{len(waypoints_body)} ---")
        print(f"[{now_str()}] Commanding BODY-frame target ({n_b:+.2f},{e_b:+.2f},{d_t:+.2f}) "
              f"→ World ({n_cmd:+.2f},{e_cmd:+.2f},{d_cmd:+.2f})")
        send_local_position_target(master, n_cmd, e_cmd, d_cmd)
        wait_until_reached(master, n_b, e_b, d_t, n0, e0, d0, yaw0, pos_tol=1.0)
        time.sleep(1.0)

    # Smooth descent before LAND
    print(f"[{now_str()}] Descending to ~0.1 m AGL before LAND ...")
    send_local_position_target(master, n0, e0, d0 - 0.1)
    wait_until_reached(master, 0, 0, -0.1, n0, e0, d0, yaw0, pos_tol=0.2, timeout_s=10)
    print(f"[{now_str()}] LAND command sent.")
    master.mav.set_mode_send(master.target_system,
                             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                             master.mode_mapping().get("LAND",0))
    time.sleep(2.0)


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Yaw-normalized autonomous square flight (body-frame mission).")
    ap.add_argument("--conn", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--alt", type=float, default=3.0)
    ap.add_argument("--side", type=float, default=5.0)
    ap.add_argument("--mode", default="GUIDED")
    args = ap.parse_args()

    def handle(sig,frame):
        print(f"\n[{now_str()}] Ctrl-C received → LAND safely.")
        sys.exit(0)
    signal.signal(signal.SIGINT, handle)

    master = connect_to_drone(args.conn, baud=args.baud)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10.0)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10.0)

    # 1) Origin + yaw calibration (on ground)
    n0,e0,d0,yaw0 = average_origin(master, duration_s=10.0)

    # 2) Arm + takeoff
    set_mode(master, args.mode)
    arm(master, True)
    takeoff(master, args.alt)

    # 3) Mission
    do_square(master, args.side, args.alt, n0,e0,d0,yaw0)

    print(f"[{now_str()}] Mission complete.")


if __name__ == "__main__":
    sys.exit(main())
