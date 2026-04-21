import time
import subprocess
import argparse
import math
from pymavlink import mavutil
from scipy.signal import max_len_seq

# Helper function to delay script execution while keeping MAVLink buffer empty
def safe_delay(delay_seconds, mav_connection):
    start_t = time.time()
    while (time.time() - start_t < delay_seconds):
        mav_connection.recv_match(blocking=False)

parser = argparse.ArgumentParser(description="Drone Drone System Identification")
parser.add_argument("--noise_dur", type=float, default=15.0, help="Duration of noise injection in seconds")
parser.add_argument("--stab_dur", type=float, default=10.0, help="Duration of stabilization phases in seconds")
parser.add_argument("--freq", type=float, default=12.0, help="Frequency of noise excitation in Hz")     
parser.add_argument("--alt", type=float, default=2.0, help="Altitude at which test will be executed")

args = parser.parse_args()

# Calculating minimal amount of samples needed to perform identyfication
required_noise_samples = math.floor(args.noise_dur / (1.0/args.freq)) + 1
i = 1
while required_noise_samples >= (2.0**i - 1):
    i+=1

# PRBS generation
noise = max_len_seq(i)
noise = noise[0] * 0.2 + 0.4

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

print("System armed")

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

# Wait for 4 seconds safely to reach altitude
safe_delay(args.stab_dur, connection)

print("Phase: Noise excitation with payload")
# Send log marker to ArduPilot .bin log
connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, b"MARKER: NOISE_1_START")

identyfication_start_time = time.time()
time_stamp = identyfication_start_time
i = 0
while (time.time() - identyfication_start_time <= args.noise_dur):
    if (time.time() - time_stamp >= 1.0/args.freq):
        # Sending noise
        connection.mav.set_attitude_target_send(
            0,
            connection.target_system,
            connection.target_component,
            7,
            [1.0, 0.0, 0.0, 0.0], # Quaternion for level flight
            0, 0, 0, 
            noise[i]  
        )
        i += 1
        time_stamp = time.time()

    connection.recv_match(blocking=False)

connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, b"MARKER: NOISE_1_STOP")

print(f"Phase: Stabilization at {args.alt} meters")
connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, b"MARKER: STABILIZATION_1")

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
        
# Phase: Payload drop
print("Phase: Payload drop")
connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, b"MARKER: PAYLOAD_DROP")
try:
    subprocess.run(
        ["gz", "topic", "-t", "/payload/detach", "-m", "gz.msgs.Empty", "-p", " "],
        check=True
    )
    print("Payload detached successfully.")
except Exception as e:
    print("Error during payload detachement:", e)

print("Phase: Stabilization after payload drop")
# Wait safely for the drone to recover from the drop
safe_delay(args.stab_dur, connection)

# Phase: Noise excitation without payload
print("Phase: Noise excitation without payload")
connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, b"MARKER: NOISE_2_START")

identyfication_start_time = time.time()
time_stamp = identyfication_start_time
i = 0
while (time.time() - identyfication_start_time <= args.noise_dur):
    if (time.time() - time_stamp >= 1.0/args.freq):
        # Sending noise
        connection.mav.set_attitude_target_send(
            0,
            connection.target_system,
            connection.target_component,
            7,
            [1.0, 0.0, 0.0, 0.0], # Quaternion for level flight
            0, 0, 0, 
            noise[i]  
        )
        i += 1
        time_stamp = time.time()

    connection.recv_match(blocking=False)

connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, b"MARKER: NOISE_2_STOP")

# Stabilize after tests
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
print("Experiment finished.")