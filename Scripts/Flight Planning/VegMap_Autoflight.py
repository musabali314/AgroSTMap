#!/usr/bin/env python3
import os, sys, cv2, math, time, signal, argparse, datetime
import numpy as np
from pymavlink import mavutil
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ------------------------------------------------------------
# Utility
# ------------------------------------------------------------
def now_str():
    return datetime.datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%S")

def make_log_dir(base="veg_logs"):
    folder = os.path.join(os.getcwd(), base, datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    os.makedirs(os.path.join(folder, "frames"), exist_ok=True)
    os.makedirs(os.path.join(folder, "heatmaps"), exist_ok=True)
    return folder

# ------------------------------------------------------------
# MAVLink control
# ------------------------------------------------------------
def connect_to_drone(conn="/dev/ttyACM0", baud=115200):
    print(f"[{now_str()}] Connecting to {conn} ...")
    master = mavutil.mavlink_connection(conn, baud=baud) if conn.startswith("/dev/") \
             else mavutil.mavlink_connection(conn)
    master.wait_heartbeat()
    print(f"[{now_str()}] Heartbeat from system {master.target_system} component {master.target_component}")
    return master

def request_stream(master, msg_id, hz=10):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0, msg_id, int(1e6/hz), 0,0,0,0,0
    )

def set_mode(master, mode="GUIDED"):
    mapping = master.mode_mapping()
    if mode not in mapping:
        print(f"[{now_str()}] Mode '{mode}' not found.")
        return False
    master.mav.set_mode_send(master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping[mode])
    print(f"[{now_str()}] Mode set to {mode}.")
    time.sleep(1)
    return True

def arm_and_takeoff(master, alt):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1,0,0,0,0,0,0)
    print(f"[{now_str()}] Arming motors ...")
    time.sleep(3)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,0,0,0,0,0,0,alt)
    print(f"[{now_str()}] Takeoff to {alt:.1f} m commanded.")
    time.sleep(6)

def send_position_target(master, n, e, d):
    mask = (1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7)|(1<<8)|(1<<11)  # pos + yaw lock
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
        n, e, d, 0,0,0, 0,0,0, 0,0)

def get_local_position(master):
    msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=True)
    return msg.x, msg.y, msg.z

def wait_until_reached(master, n_tgt, e_tgt, d_tgt, tol=0.5, timeout_s=30):
    t0=time.time()
    while True:
        n,e,d = get_local_position(master)
        dist = math.sqrt((n-n_tgt)**2 + (e-e_tgt)**2 + (d-d_tgt)**2)
        sys.stdout.write(f"\r[{now_str()}] N:{n:+.2f} E:{e:+.2f} D:{d:+.2f} err={dist:.2f} m   ")
        sys.stdout.flush()
        if dist < tol:
            print(f"\n[{now_str()}] Reached target within {tol:.1f} m.")
            return True
        if time.time()-t0 > timeout_s:
            print(f"\n[{now_str()}] Timeout reaching waypoint.")
            return False
        time.sleep(0.1)

# ------------------------------------------------------------
# ROS camera subscriber
# ------------------------------------------------------------
class CamCapture:
    def __init__(self, topic="/usb_cam/image_raw"):
        self.bridge = CvBridge()
        self.frame = None
        rospy.Subscriber(topic, Image, self._cb, queue_size=1)

    def _cb(self, msg):
        # Handles YUY2 → BGR automatically
        self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def get(self):
        return self.frame

# ------------------------------------------------------------
# Vegetation heatmap
# ------------------------------------------------------------
def compute_veg_heatmap(frame):
    """Fast green-excess vegetation heatmap."""
    frame = cv2.GaussianBlur(frame, (9,9), 0)
    b,g,r = cv2.split(frame.astype("float32"))
    denom = (r + g + b + 1e-6)
    ndg = (g - 0.5*(r+b)) / denom
    ndg = np.clip((ndg - ndg.min())/(ndg.max()-ndg.min()+1e-6), 0,1)
    heatmap = cv2.applyColorMap((ndg*255).astype(np.uint8), cv2.COLORMAP_JET)
    return heatmap

# ------------------------------------------------------------
# Main mission
# ------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Fly to point, capture frames, compute vegetation heatmap.")
    ap.add_argument("--conn", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--alt", type=float, default=10.0)
    ap.add_argument("--mode", default="GUIDED")
    ap.add_argument("--frames", type=int, default=10)
    ap.add_argument("--topic", default="/usb_cam/image_raw")
    args = ap.parse_args()

    # --- Create directories
    logdir = make_log_dir()
    print(f"[{now_str()}] Saving logs under {logdir}")

    # --- Init ROS
    rospy.init_node("veg_map_capture", anonymous=True, disable_signals=True)
    cam = CamCapture(topic=args.topic)

    # --- MAVLink setup
    master = connect_to_drone(args.conn, baud=args.baud)
    request_stream(master, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10)
    set_mode(master, args.mode)
    arm_and_takeoff(master, args.alt)

    # --- Navigate to (10,4)
    n_tgt, e_tgt, d_tgt = 10.0, 4.0, -args.alt
    send_position_target(master, n_tgt, e_tgt, d_tgt)
    wait_until_reached(master, n_tgt, e_tgt, d_tgt, tol=0.7)
    print(f"[{now_str()}] Hovering 5 s for stabilization ...")
    time.sleep(5.0)

    # --- Capture frames
    print(f"[{now_str()}] Capturing {args.frames} frames ...")
    captured = 0
    frames_dir = os.path.join(logdir, "frames")
    heatmap_dir = os.path.join(logdir, "heatmaps")
    sample_frame = None
    while captured < args.frames and not rospy.is_shutdown():
        frame = cam.get()
        if frame is None:
            time.sleep(0.1)
            continue
        fpath = os.path.join(frames_dir, f"frame_{captured:02d}.jpg")
        cv2.imwrite(fpath, frame)
        if captured == args.frames // 2:  # use mid-capture frame for heatmap
            sample_frame = frame.copy()
        print(f"[{now_str()}] Saved frame {captured+1}/{args.frames}")
        captured += 1
        time.sleep(0.4)

    # --- Compute heatmap from sample frame
    if sample_frame is not None:
        print(f"[{now_str()}] Computing vegetation heatmap ...")
        heat = compute_veg_heatmap(sample_frame)
        hpath = os.path.join(heatmap_dir, "veg_heatmap.jpg")
        cv2.imwrite(hpath, heat)
        print(f"[{now_str()}] Heatmap saved → {hpath}")

    # --- Descend to 3 m AGL
    print(f"[{now_str()}] Descending to 3 m altitude ...")
    send_position_target(master, n_tgt, e_tgt, -3.0)
    wait_until_reached(master, n_tgt, e_tgt, -3.0, tol=0.5)

    # --- Final 0.1 m AGL then LAND
    print(f"[{now_str()}] Descending to 0.1 m before LAND ...")
    send_position_target(master, n_tgt, e_tgt, -0.1)
    wait_until_reached(master, n_tgt, e_tgt, -0.1, tol=0.3)
    print(f"[{now_str()}] LAND command sent.")
    master.mav.set_mode_send(master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        master.mode_mapping().get("LAND",0))
    time.sleep(5)
    print(f"[{now_str()}] Mission complete.")

# ------------------------------------------------------------
# Run
# ------------------------------------------------------------
if __name__ == "__main__":
    signal.signal(signal.SIGINT, lambda s,f: sys.exit(0))
    sys.exit(main())
