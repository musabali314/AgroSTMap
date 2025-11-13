#!/bin/bash
# ============================================
# launch_base.sh
# Launch Pixhawk + Camera + Livox (no FAST LIO)
# ============================================

# Exit on error
set -e

echo "[INFO] Starting base drone environment..."
echo "[INFO] Make sure Mission Planner is connected via Bluetooth first."

# --- 1) Start ROS Core
echo "[INFO] Launching roscore..."
gnome-terminal -- bash -c "roscore; exec bash"
sleep 3

# --- 2) Launch MAVROS (Pixhawk connection)
echo "[INFO] Launching MAVROS (Pixhawk via USB-C)..."
gnome-terminal -- bash -c "roslaunch mavros apm.launch fcu_url:=serial:///dev/ttyACM0:115200 gcs_url:=udp://@127.0.0.1:14550; exec bash"
sleep 6

# --- 3) Publish MAVLink message streams
echo "[INFO] Setting message intervals..."
gnome-terminal -- bash -c "
rosrun mavros mavsys message_interval --id=27 --rate=200;   # RAW_IMU
rosrun mavros mavsys message_interval --id=29 --rate=200;   # SCALED_IMU2
rosrun mavros mavsys message_interval --id=30 --rate=50;    # ATTITUDE
rosrun mavros mavsys message_interval --id=74 --rate=50;    # VFR_HUD
rosrun mavros mavsys message_interval --id=0  --rate=1;     # HEARTBEAT
rosrun mavros mavsys message_interval --id=1  --rate=1;     # SYS_STATUS
rosrun mavros mavsys message_interval --id=245 --rate=1;    # EXTENDED_SYS_STATE
rosrun mavros mavsys message_interval --id=33 --rate=20;    # GLOBAL_POSITION_INT
rosrun mavros mavsys message_interval --id=32 --rate=30;    # LOCAL_POSITION_NED
rosrun mavros mavsys message_interval --id=24 --rate=5;     # GPS_RAW_INT
rosrun mavros mavsys message_interval --id=65 --rate=10;    # RC_CHANNELS
rosrun mavros mavsys message_interval --id=147 --rate=1;    # BATTERY_STATUS
rosrun mavros mavsys message_interval --id=331 --rate=30;   # ODOMETRY
exec bash"

# --- 4) Launch Camera (Arducam B0495)
echo "[INFO] Launching Arducam (YUYV @ 960x600)..."
gnome-terminal -- bash -c "rosrun usb_cam usb_cam_node _video_device:=/dev/video0 _pixel_format:=yuyv _image_width:=960 _image_height:=600 _framerate:=10; exec bash"
sleep 3

# --- 5) Launch Livox LiDAR
echo "[INFO] Launching Livox driver..."
gnome-terminal -- bash -c "cd ~/ws_livox/src/livox_ros_driver2/launch_ROS1 && roslaunch msg_MID360.launch; exec bash"
sleep 4

# --- 6) Health Checks
echo "[INFO] Checking topic rates..."
gnome-terminal -- bash -c "
rostopic hz /livox/imu &
rostopic hz /livox/lidar &
rostopic hz /usb_cam/image_raw &
exec bash"

echo "[OK] Base setup running."
echo "[Next] Run your mission script (e.g. python3 field_glide.py) in another terminal."
