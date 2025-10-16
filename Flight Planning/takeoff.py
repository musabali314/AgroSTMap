import time
import math
from pymavlink import mavutil

# --- Configuration ---
# Set the target altitude for takeoff in meters
TAKEOFF_ALTITUDE = 5.0
# Set the time to wait after takeoff command before checking altitude
TAKEOFF_TIMEOUT = 30
# Set the connection string for your flight controller
# UDP is common for companion computers
CONNECTION_STRING = 'udpin:127.0.0.1:14550'

# --- Vision System Placeholder ---
def get_vision_pose():
    """
    Placeholder function for your vision system.
    Replace this with your actual code to get drone's pose (x, y, z, roll, pitch, yaw)
    relative to a known reference frame (e.g., AprilTag).
    Returns a tuple (x, y, z, roll, pitch, yaw) and a boolean indicating success.
    All position values are in meters, attitude in radians.
    """
    # YOUR CODE HERE
    # Example: Return dummy data for testing
    current_time_ms = time.time() * 1000
    x, y, z = 0.0, 0.0, 0.0
    roll, pitch, yaw = 0.0, 0.0, 0.0

    # In a real scenario, this would be a constant stream of valid data
    # For now, let's assume we have a valid pose
    return (x, y, z, roll, pitch, yaw), True

# --- MAVLink Helper Functions ---
def wait_for_heartbeat(the_connection):
    """Wait for a heartbeat from the FCU."""
    print("Waiting for heartbeat...")
    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

def arm_and_takeoff(the_connection, altitude):
    """
    Sends arm and takeoff commands to the FCU.
    """
    print("Arming and taking off to %.1f meters..." % altitude)

    # Set mode to GUIDED
    print("Setting mode to GUIDED...")
    the_connection.set_mode('GUIDED')

    # Wait for mode change to be acknowledged (optional but good practice)
    # the_connection.recv_match(type='HEARTBEAT', blocking=True, timeout=5)

    # Arm the drone
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)

    # Wait for arming to be confirmed
    ack_message = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if not (ack_message and ack_message.result == mavutil.mavlink.MAV_RESULT_ACCEPTED):
        print("Arming failed. Aborting.")
        return False
    print("Arming successful.")

    # Send takeoff command
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude)

    return True
    
def check_altitude_reached(the_connection, target_alt, tolerance=0.5):
    """
    Checks if the drone has reached the target altitude.
    """
    msg = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if msg:
        # The MAVLink protocol specifies relative_alt as negative for altitude above home.
        # We need to convert it to a positive value for comparison.
        current_alt = -msg.relative_alt / 1000.0  # Convert mm to m and make positive
        print("Current altitude: %.2f m" % current_alt)
        # Compare the absolute value of current altitude to the target altitude
        return abs(current_alt) > (target_alt - tolerance)
    return False

def send_vision_position(the_connection, x, y, z, roll, pitch, yaw):
    """
    Sends vision position estimate to the FCU.
    """
    timestamp = int(time.time() * 1e6)  # Timestamp in microseconds
    the_connection.mav.vision_position_estimate_send(
        timestamp,
        x, y, z,
        roll, pitch, yaw
    )

# --- Main Script Logic ---
def main():
    try:
        # Create a MAVLink connection
        the_connection = mavutil.mavlink_connection(CONNECTION_STRING)
        wait_for_heartbeat(the_connection)

        # Main loop for vision data streaming and flight control
        script_start_time = time.time()  # New variable to track total script runtime
        takeoff_initiated = False
        takeoff_complete = False
        takeoff_start_time = 0  # Initialize takeoff start time

        while True:
            # Get data from your vision system
            (x, y, z, roll, pitch, yaw), vision_ok = get_vision_pose()

            if not vision_ok:
                print("Vision system not providing valid data. Retrying...")
                time.sleep(1)
                continue

            # Send vision position to the FCU at a high rate
            send_vision_position(the_connection, x, y, z, roll, pitch, yaw)

            # --- Autonomous Flight Logic ---
            if not takeoff_initiated:
                print("Waiting for stable vision data before arming...")
                if time.time() - script_start_time > 5:  # Wait 5 seconds for visual pose to be stable
                    if arm_and_takeoff(the_connection, TAKEOFF_ALTITUDE):
                        takeoff_initiated = True
                        takeoff_start_time = time.time()  # Set takeoff start time here
                    else:
                        print("Exiting due to arming failure.")
                        break

            elif not takeoff_complete:
                # Check if takeoff altitude has been reached
                if check_altitude_reached(the_connection, TAKEOFF_ALTITUDE):
                    print("Takeoff successful! Reached target altitude.")
                    takeoff_complete = True
                elif time.time() - takeoff_start_time > TAKEOFF_TIMEOUT:
                    print("Takeoff timed out. Aborting mission.")
                    the_connection.set_mode('LAND') # Failsafe
                    break

            else:
                # --- YOUR MISSION LOGIC HERE ---
                # Example: Hover for 10 seconds, then land
                if time.time() - takeoff_start_time < 40:
                    print("Hovering at target altitude...")
                    the_connection.mav.set_position_target_local_ned_send(
                        0, the_connection.target_system, the_connection.target_component,
                        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                        0b0000111111111000,
                        0, 0, -TAKEOFF_ALTITUDE,
                        0, 0, 0,
                        0, 0, 0, 0, 0, 0
                    )
                else:
                    print("Mission complete. Landing...")
                    the_connection.set_mode('LAND')
                    break

            time.sleep(0.01) # Loop rate

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        if 'the_connection' in locals():
            print("Closing connection...")
            the_connection.close()
            print("Connection closed.")

if __name__ == '__main__':
    main()