import time
import argparse
import subprocess
from pymavlink import mavutil

# Helper function to delay script execution while keeping MAVLink buffer empty
def safe_delay(delay_seconds, mav_connection):
    start_t = time.time()
    while (time.time() - start_t < delay_seconds):
        mav_connection.recv_match(blocking=False)

#  HELPER FUNCTION: Continuously stream attitude/thrust targets to prevent ArduPilot timeout
def execute_thrust_step(duration, thrust_val, mav_connection, update_freq=50.0):
    start_t = time.time()
    last_send_t = 0
    update_interval = 1.0 / update_freq
    
    while (time.time() - start_t < duration):
        current_t = time.time()
        # Send command at the specified frequency
        if (current_t - last_send_t >= update_interval):
            mav_connection.mav.set_attitude_target_send(
                0,                                  # time_boot_ms (0 means ignore)
                mav_connection.target_system,       # target_system
                mav_connection.target_component,    # target_component
                7,                                  # type_mask (ignore rates, use attitude and thrust)
                [1.0, 0.0, 0.0, 0.0],               # q: Quaternion for level flight
                0, 0, 0,                            # body roll, pitch, yaw rates
                thrust_val                          # thrust
            )
            last_send_t = current_t
            
        # Keep buffer clean
        mav_connection.recv_match(blocking=False)


parser = argparse.ArgumentParser(description="Drone System Identification Validation")
parser.add_argument("--ver_dur", type=float, default=4.0, help="Duration of validation process in seconds")
parser.add_argument("--stab_dur", type=float, default=5.0, help="Duration of stabilization phases in seconds")
parser.add_argument("--alt", type=float, default=2.0, help="Altitude at which test will be executed")
parser.add_argument("--thro", type=float, default=0.55, help="Value of normalized throttle (0.5 is hover point) which is stimuli to the system")

args = parser.parse_args()

# Connect to the simulator
connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# Wait for the first heartbeat
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

try:
    mode_id = connection.mode_mapping()['GUIDED']
except Exception as e:
    print("Cannot download mode id from flight controller")
    exit()

# Arming loop
while not connection.motors_armed():
    # Set to GUIDED mode
    connection.set_mode(mode_id)
    # Send arm command
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, # 1 means Arm
        0, 0, 0, 0, 0, 0
    )
    print("System disarmed, waiting for system setup...")
    safe_delay(2.0, connection)

# Send autonomous takeoff command
connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, # Confirmation
    0, 0, 0, 0, 0, 0, 
    args.alt
)
print(f"Takeoff command sent. Ascending to {args.alt} meters.")

# Wait safely for the drone to reach the target altitude
safe_delay(args.stab_dur, connection)

# PHASE 1: Step stimuli with payload
print("Phase: Step stimuli with payload")

# Send log marker to ArduPilot .bin log
connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, b"MARKER: START_STEP_PAYLOAD")

# Execute continuous thrust step
execute_thrust_step(args.ver_dur, args.thro, connection)

# Send stop marker
connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, b"MARKER: STOP_STEP_PAYLOAD")

# PHASE 2: Stabilization at target altitude
print(f"Phase: Stabilization at {args.alt} meters")
connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, b"MARKER: STABILIZATION_1")

# Command drone back to the specific altitude and position
connection.mav.set_position_target_local_ned_send(
    0,                                  # time_boot_ms
    connection.target_system,           
    connection.target_component,        
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,# coordinate_frame
    2552,                               # type_mask (ignore velocities and accelerations)
    0, 0, -args.alt,                    # x, y, z (z is negative in NED)
    0, 0, 0,                            # vx, vy, vz
    0, 0, 0,                            # afx, afy, afz
    0, 0                                # yaw, yaw_rate
)

# Wait safely for stabilization
safe_delay(args.stab_dur, connection)

# PHASE 3: Payload drop
print("Phase: Payload drop")
connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, b"MARKER: PAYLOAD_DROP")
try:
    subprocess.run(
        ["gz", "topic", "-t", "/payload/detach", "-m", "gz.msgs.Empty", "-p", " "],
        check=True
    )
    print("Payload detached successfully.")
except Exception as e:
    print("Error during payload detachment:", e)

print("Phase: Stabilization after payload drop")
# Wait safely for the drone to recover from the drop disturbance
safe_delay(args.stab_dur, connection)

# PHASE 4: Step stimuli without payload
print("Phase: Step stimuli without payload")

# Send log marker to ArduPilot .bin log
connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, b"MARKER: START_STEP_NO_PAYLOAD")

# Execute continuous thrust step
execute_thrust_step(args.ver_dur, args.thro, connection)

# Send stop marker
connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, b"MARKER: STOP_STEP_NO_PAYLOAD")

# ==========================================
# PHASE 5: Final Stabilization
# ==========================================
print(f"Phase: Stabilization at {args.alt} meters")
connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, b"MARKER: STABILIZATION_2")

# Command drone back to altitude
connection.mav.set_position_target_local_ned_send(
    0,                                  # time_boot_ms
    connection.target_system,           
    connection.target_component,        
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,# coordinate_frame
    2552,                               # type_mask
    0, 0, -args.alt,                    
    0, 0, 0,                            
    0, 0, 0,                            
    0, 0                                
)

# Final delay to let the drone stabilize before ending script
safe_delay(2.0, connection)
print("Validation data collecting finished!")