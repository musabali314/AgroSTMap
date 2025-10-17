#!/usr/bin/env python3
import argparse
import math
import os
import signal
import sys
import time
from datetime import datetime

import rospy
from nav_msgs.msg import Odometry
from pymavlink import mavutil

# -----------------------------
# Helpers (unchanged)
# -----------------------------

def now_str():
    return datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%S.%fZ")

def is_serial(conn: str) -> bool:
    return conn.startswith("/dev/")

def request_message_interval(master, msg_id: int, hz: float) -> None:
    if hz <= 0:
        return
    interval_us = int(1e6 / hz)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        msg_id,
        interval_us,
        0, 0, 0, 0, 0,
    )

# -----------------------------
# Core FCU operations (unchanged)
# -----------------------------

def connect_to_drone(connection_string: str, baud: int = 115200):
    print(f"[{now_str()}] Connecting to {connection_string} ...")
    if is_serial(connection_string):
        master = mavutil.mavlink_connection(connection_string, baud=baud)
    else:
        master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    print(f"[{now_str()}] Heartbeat received from system {master.target_system} component {master.target_component}")
    return master

def set_mode(master, mode: str) -> bool:
    mapping = master.mode_mapping()
    if mode not in mapping:
        print(f"[{now_str()}] Mode '{mode}' not in mode_mapping: {mapping}.")
        return False
    mode_id = mapping[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    time.sleep(1.0)
    print(f"[{now_str()}] Mode set to {mode}.")
    return True

def arm(master, arm_it: bool = True) -> None:
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1 if arm_it else 0, 0, 0, 0, 0, 0, 0
    )
    print(f"[{now_str()}] {'Arming' if arm_it else 'Disarming'} command sent.")
    time.sleep(2.0)

def takeoff(master, altitude_m: float) -> None:
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude_m
    )
    print(f"[{now_str()}] Takeoff to {altitude_m:.2f} m commanded.")
    time.sleep(3.0)

def send_local_position_target(master, n: float, e: float, d: float) -> None:
    type_mask_pos_only = 0b0000111111111000
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask_pos_only,
        n, e, d,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )

def get_local_position(master):
    msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=True)
    return (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)

# -----------------------------
# New: velocity control helpers (slow & stable)
# -----------------------------
def send_local_velocity_ned(master, vn: float, ve: float, vd: float, yaw_rate: float = 0.0) -> None:
    """Send velocity-only setpoint in LOCAL_NED.
    vn, ve, vd are m/s (north, east, down). Keep small for smooth motion.
    """
    type_mask_vel_only = 0b0000111000000111  # decimal 3527
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask_vel_only,
        0, 0, 0,
        vn, ve, vd,
        0, 0, 0,
        0, 0
    )

def brake(master, duration_s: float = 0.5):
    t0 = time.time()
    while time.time() - t0 < duration_s:
        send_local_velocity_ned(master, 0.0, 0.0, 0.0)
        time.sleep(0.05)

# -----------------------------
# FAST-LIO subscriber (no EKF, just position deltas)
# -----------------------------
class FastLioTracker:
    """Consumes /Odometry from FAST-LIO, converts to NED using mapping:
        x->down (D = +x), y->left (E = -y), z->forward (N = +z)
    Provides zeroed coordinates and leg-based deltas.
    """
    def __init__(self, topic="/Odometry"):
        self.topic = topic
        self._have = False
        self._n = self._e = self._d = 0.0
        self._n0 = self._e0 = self._d0 = None
        self._leg_start = None
        rospy.Subscriber(self.topic, Odometry, self._cb, queue_size=50)

    def _cb(self, msg: Odometry):
        x = msg.pose.pose.position.x   # down (+)
        y = msg.pose.pose.position.y   # left (+)
        z = msg.pose.pose.position.z   # forward (+)
        n = z
        e = -y
        d = x

        if self._n0 is None:
            self._n0, self._e0, self._d0 = n, e, d
            self._leg_start = (n, e, d)
            rospy.loginfo("[%s] FAST-LIO zero set (NED) to (%.3f, %.3f, %.3f)", now_str(), n, e, d)

        self._n = n - self._n0
        self._e = e - self._e0
        self._d = d - self._d0
        self._have = True

    def wait_ready(self, timeout_s=5.0):
        t0 = time.time()
        while not self._have:
            if time.time() - t0 > timeout_s:
                raise RuntimeError("FAST-LIO /Odometry not received in time")
            if rospy.is_shutdown():
                raise RuntimeError("ROS shutdown")
            time.sleep(0.02)

    def start_leg(self):
        self._leg_start = (self._n, self._e, self._d)

    def delta_since_leg(self):
        if self._leg_start is None:
            return (0.0, 0.0, 0.0)
        dn = self._n - self._leg_start[0]
        de = self._e - self._leg_start[1]
        dd = self._d - self._leg_start[2]
        return (dn, de, dd)

    def current_ned(self):
        return (self._n, self._e, self._d)

# -----------------------------
# Mode watcher (RC takeover)
# -----------------------------
def current_mode(master):
    hb = master.recv_match(type='HEARTBEAT', blocking=False)
    if not hb:
        return None
    mapping = master.mode_mapping()
    rev = {v: k for k, v in mapping.items()}
    return rev.get(hb.custom_mode, str(hb.custom_mode))

# -----------------------------
# Leg execution driven by FAST-LIO deltas
# -----------------------------
def run_leg_with_lio(master, lio: FastLioTracker, dn_goal=0.0, de_goal=0.0, dd_goal=0.0,
                     speed_mps=0.5, pos_tol=0.15, timeout_s=30.0, expected_mode="GUIDED"):
    total = math.sqrt(dn_goal**2 + de_goal**2 + dd_goal**2)
    if total < 1e-6:
        return True
    vn = speed_mps * (dn_goal / total)
    ve = speed_mps * (de_goal / total)
    vd = speed_mps * (dd_goal / total)

    lio.start_leg()
    t0 = time.time()
    last_send = 0.0
    while True:
        mode = current_mode(master)
        if mode and mode != expected_mode:
            print(f"\n[{now_str()}] Pilot took over (mode={mode}). Stopping leg.")
            brake(master, 0.4)
            return False

        if time.time() - last_send > 1.0 / 10.0:
            send_local_velocity_ned(master, vn, ve, vd)
            last_send = time.time()

        dn, de, dd = lio.delta_since_leg()
        err = math.sqrt((dn_goal - dn)**2 + (de_goal - de)**2 + (dd_goal - dd)**2)
        sys.stdout.write(
            f"\r[{now_str()}] LIO dNED=({dn:+.2f},{de:+.2f},{dd:+.2f}) "
            f"target=({dn_goal:+.2f},{de_goal:+.2f},{dd_goal:+.2f}) err={err:.2f} m   "
        )
        sys.stdout.flush()

        if abs(total - math.sqrt(dn*dn + de*de + dd*dd)) <= pos_tol or err <= pos_tol:
            print(f"\n[{now_str()}] Leg reached within {pos_tol} m (FAST-LIO).")
            brake(master, 0.6)
            time.sleep(1.0)  # settle
            return True

        if time.time() - t0 > timeout_s:
            print(f"\n[{now_str()}] Leg timeout ({timeout_s}s).")
            brake(master, 0.6)
            return False

        time.sleep(0.02)

# -----------------------------
# Mission: 5 m square using FAST-LIO deltas only
# -----------------------------
def do_lio_square(master, lio: FastLioTracker, side_m: float, alt_m: float, speed_mps: float, expected_mode="GUIDED"):
    # Hold altitude using a position target at current N/E and D = -alt
    n, e, d, *_ = get_local_position(master)
    target_d = -alt_m
    for _ in range(30):
        send_local_position_target(master, n, e, target_d)
        time.sleep(0.05)

    print(f"[{now_str()}] Starting FAST-LIO-driven square, side={side_m} m, speed={speed_mps} m/s")

    ok1 = run_leg_with_lio(master, lio, dn_goal=+side_m, de_goal=0.0, dd_goal=0.0,
                           speed_mps=speed_mps, expected_mode=expected_mode)
    ok2 = run_leg_with_lio(master, lio, dn_goal=0.0, de_goal=+side_m, dd_goal=0.0,
                           speed_mps=speed_mps, expected_mode=expected_mode)
    ok3 = run_leg_with_lio(master, lio, dn_goal=-side_m, de_goal=0.0, dd_goal=0.0,
                           speed_mps=speed_mps, expected_mode=expected_mode)
    ok4 = run_leg_with_lio(master, lio, dn_goal=0.0, de_goal=-side_m, dd_goal=0.0,
                           speed_mps=speed_mps, expected_mode=expected_mode)

    print(f"[{now_str()}] Legs: {ok1}, {ok2}, {ok3}, {ok4}. Commanding LAND...")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        master.mode_mapping().get("LAND", 0)
    )
    time.sleep(1.0)

# -----------------------------
# Main
# -----------------------------
def main():
    ap = argparse.ArgumentParser(description="FAST-LIO driven 4-leg square (no EKF; slow velocity control).")
    ap.add_argument("--conn", default="udpin:localhost:14550", help="MAVLink connection string")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mode", default="GUIDED")
    ap.add_argument("--alt", type=float, default=2.0, help="Altitude (m AGL)")
    ap.add_argument("--side", type=float, default=5.0, help="Square side length (m)")
    ap.add_argument("--speed", type=float, default=0.4, help="Horizontal speed (m/s)")
    ap.add_argument("--lio_topic", default="/Odometry")
    args = ap.parse_args()

    # Init ROS (just for FAST-LIO subscriber)
    rospy.init_node("lio_square_controller", anonymous=True, disable_signals=True)
    lio = FastLioTracker(topic=args.lio_topic)
    print(f"[{now_str()}] Waiting for FAST-LIO /Odometry ...")
    lio.wait_ready(timeout_s=10.0)

    # MAVLink connect & prep
    master = connect_to_drone(args.conn, baud=args.baud)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10.0)
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10.0)

    if not set_mode(master, args.mode):
        print(f"[{now_str()}] Failed to set mode {args.mode}. Aborting.")
        return 2

    arm(master, True)
    time.sleep(1.0)
    takeoff(master, args.alt)
    time.sleep(2.0)

    # Hold altitude at the start
    n, e, d, *_ = get_local_position(master)
    for _ in range(30):
        send_local_position_target(master, n, e, -args.alt)
        time.sleep(0.05)

    # Fly the square via FAST-LIO deltas
    do_lio_square(master, lio, side_m=args.side, alt_m=args.alt, speed_mps=args.speed, expected_mode=args.mode)

    print(f"[{now_str()}] Mission complete.")

def handle_sigint(sig, frame):
    print(f"\n[{now_str()}] Ctrl+C received. Flip RC to AltHold/RTL to take over immediately.")

if __name__ == "__main__":
    signal.signal(signal.SIGINT, handle_sigint)
    sys.exit(main())
