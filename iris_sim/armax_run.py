#!/usr/bin/env python3
"""
============================================================
ARMAX Automated Run - Uniform Data Collection Pipeline
============================================================
Connects to the running Gazebo simulation, logs IMU data,
triggers the payload drop at a precise moment, and saves a
clean CSV ready for MATLAB ARMAX analysis.

This script ensures EVERY run is identical:
  - Same hover stabilisation time
  - Same drop timing
  - Same total recording duration
  - Same sample rate
  - Same CSV format

Prerequisites:
  - Gazebo running with iris drone hovering at 5m
  - SITL connected (drone armed in GUIDED mode at altitude)

Usage:
  python3 armax_run.py 1     # Run number (used in filename)
  python3 armax_run.py 2
  python3 armax_run.py 3
============================================================
"""
from gz.transport13 import Node
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.empty_pb2 import Empty
import csv, time, datetime, math, os, sys, threading

# ── Experiment Configuration ──────────────────────────────
SAMPLE_RATE_HZ      = 100              # 100 Hz logging
SAMPLE_INTERVAL     = 1.0 / SAMPLE_RATE_HZ
PRE_DROP_DURATION   = 5.0              # seconds of stable hover before drop
POST_DROP_DURATION  = 25.0             # seconds of recovery after drop
TOTAL_DURATION      = PRE_DROP_DURATION + POST_DROP_DURATION

IMU_TOPIC = ('/world/armax_hover/model/iris_with_ardupilot'
             '/model/iris_with_standoffs/link/imu_link'
             '/sensor/imu_sensor/imu')
DROP_TOPIC = '/payload_drop'

# ── Parse command line ────────────────────────────────────
if len(sys.argv) < 2:
    print("Usage: python3 armax_run.py <run_number>")
    sys.exit(1)

run_num = int(sys.argv[1])

# ── State ─────────────────────────────────────────────────
imu = {'roll':0.0, 'pitch':0.0, 'yaw':0.0,
       'wx':0.0, 'wy':0.0, 'wz':0.0}

def imu_callback(msg):
    q = msg.orientation
    sinr = 2.0*(q.w*q.x + q.y*q.z)
    cosr = 1.0 - 2.0*(q.x**2 + q.y**2)
    imu['roll'] = math.degrees(math.atan2(sinr, cosr))
    sinp = max(-1.0, min(1.0, 2.0*(q.w*q.y - q.z*q.x)))
    imu['pitch'] = math.degrees(math.asin(sinp))
    siny = 2.0*(q.w*q.z + q.x*q.y)
    cosy = 1.0 - 2.0*(q.y**2 + q.z**2)
    imu['yaw'] = math.degrees(math.atan2(siny, cosy))
    imu['wx'] = math.degrees(msg.angular_velocity.x)
    imu['wy'] = math.degrees(msg.angular_velocity.y)
    imu['wz'] = math.degrees(msg.angular_velocity.z)

# ── Connect to Gazebo ─────────────────────────────────────
node = Node()
node.subscribe(IMU, IMU_TOPIC, imu_callback)
drop_pub = node.advertise(DROP_TOPIC, Empty)

print()
print("=" * 60)
print(f"  ARMAX Automated Run #{run_num}")
print("=" * 60)
print(f"  Total duration: {TOTAL_DURATION}s")
print(f"  Drop trigger at: t = {PRE_DROP_DURATION}s")
print(f"  Sample rate: {SAMPLE_RATE_HZ} Hz")
print()

print("  Waiting for IMU connection (2s)...")
time.sleep(2.0)

input("  Press ENTER when drone is hovering stably at 5m to begin...\n")

# ── Setup output ──────────────────────────────────────────
ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
filepath = os.path.expanduser(
    f"~/armax_data/run{run_num:02d}_{ts}.csv")
os.makedirs(os.path.dirname(filepath), exist_ok=True)

print(f"  Logging to: {filepath}")
print()

# ── Drop trigger (background thread) ──────────────────────
drop_triggered = [False]
drop_sample_idx = [None]

def trigger_drop_at_time(target_time):
    while time.time() < target_time:
        time.sleep(0.001)
    msg = Empty()
    drop_pub.publish(msg)
    drop_triggered[0] = True
    print(f"  >>> PAYLOAD DROP TRIGGERED at t={target_time-start_time:.3f}s <<<")

# ── Main logging loop ────────────────────────────────────
print("  Starting data collection...")
print()
print(f"  {'Time':>8} {'Phase':>10} {'Roll':>10} {'Pitch':>10} {'Yaw':>10}")
print("  " + "-"*54)

start_time = time.time()
drop_target_time = start_time + PRE_DROP_DURATION

# Schedule drop in background thread
drop_thread = threading.Thread(
    target=trigger_drop_at_time,
    args=(drop_target_time,),
    daemon=True)
drop_thread.start()

samples = []
last_print = 0.0
sample_idx = 0

while time.time() - start_time < TOTAL_DURATION:
    target_t = sample_idx * SAMPLE_INTERVAL
    elapsed = time.time() - start_time

    if elapsed >= target_t:
        # Determine phase
        if elapsed < PRE_DROP_DURATION:
            phase = "PRE_DROP"
            drop_event = 0
        elif drop_sample_idx[0] is None and drop_triggered[0]:
            phase = "DROP"
            drop_event = 1
            drop_sample_idx[0] = sample_idx
        elif elapsed < PRE_DROP_DURATION + 1.0:
            phase = "DROP"
            drop_event = 0
        else:
            phase = "POST_DROP"
            drop_event = 0

        samples.append({
            'time_s': round(target_t, 4),
            'phase': phase,
            'roll_deg': round(imu['roll'], 6),
            'pitch_deg': round(imu['pitch'], 6),
            'yaw_deg': round(imu['yaw'], 6),
            'rollspeed_deg_s': round(imu['wx'], 6),
            'pitchspeed_deg_s': round(imu['wy'], 6),
            'yawspeed_deg_s': round(imu['wz'], 6),
            'drop_event': drop_event
        })

        # Console output every 0.5s
        if elapsed - last_print >= 0.5:
            print(f"  {elapsed:7.2f}s [{phase:>10}] "
                  f"{imu['roll']:9.3f}° {imu['pitch']:9.3f}° "
                  f"{imu['yaw']:9.3f}°"
                  f"{'  *DROP*' if drop_event else ''}")
            last_print = elapsed

        sample_idx += 1

    time.sleep(0.0005)

# ── Save CSV ──────────────────────────────────────────────
print()
print("  Writing CSV...")

with open(filepath, 'w', newline='') as f:
    f.write(f"# ARMAX Iris Drone Run #{run_num}\n")
    f.write(f"# Recorded: {datetime.datetime.now()}\n")
    f.write(f"# Sample rate: {SAMPLE_RATE_HZ} Hz "
            f"(Ts = {SAMPLE_INTERVAL}s)\n")
    f.write(f"# Pre-drop hover: {PRE_DROP_DURATION}s\n")
    f.write(f"# Post-drop log:  {POST_DROP_DURATION}s\n")
    f.write(f"# Drop sample idx: {drop_sample_idx[0]}\n")
    f.write(f"# MATLAB: y=roll_deg, u=drop_event, "
            f"Ts={SAMPLE_INTERVAL}\n")
    f.write("#\n")

    fieldnames = ['time_s', 'phase', 'roll_deg', 'pitch_deg',
                  'yaw_deg', 'rollspeed_deg_s', 'pitchspeed_deg_s',
                  'yawspeed_deg_s', 'drop_event']
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(samples)

print()
print("=" * 60)
print(f"  Run #{run_num} COMPLETE")
print(f"  Samples: {len(samples)}")
print(f"  Saved: {filepath}")
print("=" * 60)
