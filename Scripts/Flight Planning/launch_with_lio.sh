#!/bin/bash
# ============================================
# launch_with_lio.sh
# Launch Pixhawk + Camera + Livox + FAST LIO
# ============================================

set -e
echo "[INFO] Starting full LIO-enabled environment..."

# --- 1) Start ROS Core
echo "[INFO] Launching roscore..."
gnome-terminal -- bash -c "roscore; exec bash"
sleep 3

# --- 2) Launch MAVROS
echo "[INFO] Launching MAVROS (Pixhawk via USB-C)..."
gnome-terminal -- bash -c "roslaunch mavros apm.launch fcu_url:=serial:///dev/ttyACM0:115200 gcs_url:=udp://@127.0.0.1:14550; exec bash"
sleep 6

# --- 3) Publish message intervals
echo "[INFO] Setting MAVLink message intervals..."
gnome-terminal -- bash -c "
rosrun mavros mavsys message_interval --id=27 --rate=200;
rosrun mavros mavsys message_interval --id=29 --rate=200;
rosrun mavros mavsys message_interval --id=30 --rate=50;
rosrun mavros mavsys message_interval --id=74 --rate=50;
rosrun mavros mavsys message_interval --id=0  --rate=1;
rosrun mavros mavsys message_interval --id=1  --rate=1;
rosrun mavros mavsys message_interval --id=245 --rate=1;
rosrun mavros mavsys message_interval --id=33 --rate=20;
rosrun mavros mavsys message_interval --id=32 --rate=30;
rosrun mavros mavsys message_interval --id=24 --rate=5;
rosrun mavros mavsys message_interval --id=65 --rate=10;
rosrun mavros mavsys message_interval --id=147 --rate=1;
rosrun mavros mavsys message_interval --id=331 --rate=30;
exec bash"

# --- 4) Launch Camera
echo "[INFO] Launching Arducam (YUYV @ 960x600)..."
gnome-terminal -- bash -c "rosrun usb_cam usb_cam_node _video_device:=/dev/video0 _pixel_format:=yuyv _image_width:=960 _image_height:=600 _framerate:=10; exec bash"
sleep 3

# --- 5) Launch Livox LiDAR
echo "[INFO] Launching Livox driver..."
gnome-terminal -- bash -c "cd ~/ws_livox/src/livox_ros_driver2/launch_ROS1 && roslaunch msg_MID360.launch; exec bash"
sleep 4

# --- 6) Launch FAST LIO (after Livox up)
echo "[INFO] Launching FAST-LIO mapping..."
gnome-terminal -- bash -c "cd ~/fastlio_ws/src/FAST_LIO/launch && roslaunch mapping_mid360.launch; exec bash"
sleep 4

# --- 7) Topic checks
echo "[INFO] Checking topic rates..."
gnome-terminal -- bash -c "
rostopic hz /livox/imu &
rostopic hz /livox/lidar &
rostopic hz /usb_cam/image_raw &
rostopic hz /Odometry &
exec bash"

echo "[OK] LIO-enabled setup running."
echo "[Next] Run your mission script (e.g. python3 lio_gps_fused.py) in another terminal."
