# Iris Drone ARMAX System Identification

System identification of an Iris quadrotor's attitude response to a payload-drop disturbance, using ArduPilot SITL + Gazebo Harmonic for simulation and MATLAB ARMAX for analysis.

---

## What This Is

A complete pipeline for:

1. **Simulating** an Iris drone in Gazebo Harmonic with a 500 g detachable payload
2. **Recording** the IMU response (roll, pitch, yaw) at 100 Hz during a payload drop
3. **Identifying** ARMAX models of the disturbance dynamics in MATLAB
4. **Validating** the models with cross-validation, residual whiteness, pole-zero stability, and Gaussian-noise robustness tests

---

## Repository Layout

```
iris_sim/
├── README.md                          # This file
├── armax_hover.sdf                    # Gazebo world (drone + ground plane)
├── iris_with_ardupilot_model.sdf      # Iris drone with gripper + payload
├── armax_run.py                       # Automated single-run data collector
├── armax_logger.py                    # Manual logger (legacy, kept for reference)
├── armax_v5.m                         # Comprehensive MATLAB ARMAX analysis
└── REPORT.md                          # Brief technical report
```

---

## Prerequisites

| Software | Version | Notes |
|----------|---------|-------|
| Ubuntu | 22.04 | tested |
| Gazebo Harmonic | gz-sim 8.11.0 | `sudo apt install gz-harmonic` |
| ArduPilot SITL | ArduCopter 4.6 | `~/ardupilot/build/sitl/bin/arducopter` |
| ardupilot_gazebo plugin | main branch | rebuilt for Harmonic |
| MATLAB | R2025b | with System Identification + Statistics Toolboxes |
| Python 3 | 3.10+ | with `gz-transport13` bindings |

Required Python packages:
```bash
sudo apt install python3-gz-transport13 python3-gz-msgs10
```

---

## Setup

### One-time: Place the model and world files

```bash
# Copy iris drone + payload model
cp iris_sim/iris_with_ardupilot_model.sdf \
   ~/ardupilot_gazebo/models/iris_with_ardupilot/model.sdf

# Copy world file
cp iris_sim/armax_hover.sdf \
   ~/ardupilot_gazebo/worlds/armax_hover.sdf

# Copy logger script to home
cp iris_sim/armax_run.py ~/armax_run.py
chmod +x ~/armax_run.py

# Copy MATLAB script
cp iris_sim/armax_v5.m ~/armax_v5.m
```

---

## How to Run a Data Collection Cycle

### Terminal 1 — SITL (ArduCopter)

```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py \
  -v ArduCopter -f gazebo-iris \
  --model JSON --console --no-rebuild -I 0
```

Wait for `Waiting for connection ....`

### Terminal 2 — Gazebo

```bash
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build
gz sim -r ~/ardupilot_gazebo/worlds/armax_hover.sdf
```

You should see the drone with a payload box held by a gripper underneath.

### MAVProxy Xterm window — Arm and takeoff

```
param set ARMING_SKIPCHK 1
param set FS_EKF_ACTION 0
param set TERRAIN_ENABLE 0
mode GUIDED
arm throttle force
takeoff 5
```

Wait for the drone to stabilise at 5 m altitude.

### Terminal 3 — Run a single data collection

```bash
python3 ~/armax_run.py 1
```

Press **ENTER** when the drone is hovering stably. The script:
- Logs IMU at 100 Hz for 5 seconds (pre-drop baseline)
- Auto-triggers `/payload_drop` at exactly t = 5s
- Continues logging for 25 seconds (post-drop recovery)
- Saves CSV to `~/armax_data/run01_YYYYMMDD_HHMMSS.csv`
- **Stops automatically** at t = 30 s

Total recording: 30 seconds, 3000 samples, identical every run.

### Repeat for multiple runs

For each new run, restart Gazebo and SITL (the payload only attaches once per Gazebo session). Then:

```bash
python3 ~/armax_run.py 2
python3 ~/armax_run.py 3
python3 ~/armax_run.py 4
python3 ~/armax_run.py 5
```

5 runs are enough for a solid ARMAX fit.

---

## How to Run the MATLAB Analysis

```bash
matlab -r "run('~/armax_v5.m')"
```

Or open MATLAB and type:

```matlab
cd ~
armax_v5
```

The script automatically discovers every `run*.csv` in `~/armax_data/` and runs the full pipeline.

### Output

**Console summary:**
```
=================================================
          ARMAX ANALYSIS SUMMARY
=================================================
  Runs analyzed:       5 (train: 3, val: 2)
  Best orders:         ARMAX[5 4 3 1]
  TRAINING FIT:
    Roll:  92.71 %
    Pitch: 95.56 %
    Yaw:   62.20 %
  ...
```

**7 figures:**
1. Grid-search heatmaps for order selection
2. ARMAX model vs measured data (roll/pitch/yaw)
3. Residual analysis
4. Noise-robustness bar charts (3 noise levels)
5. Bode response (clean vs noisy models)
6. Pole-zero map
7. Raw IMU data overview

**Saved file:**
`armax_models.mat` — contains identified models for further use

---

## Configuration

### `armax_run.py`

```python
SAMPLE_RATE_HZ      = 100              # logging rate
PRE_DROP_DURATION   = 5.0              # baseline hover seconds
POST_DROP_DURATION  = 25.0             # recovery seconds
```

### `armax_v5.m`

```matlab
DATA_DIR    = 'YOUR FILE DIRECTORY';
Ts          = 0.01;                    % must match SAMPLE_RATE_HZ
NOISE_LOW   = 0.005;                   % degrees, 3 noise levels
NOISE_MED   = 0.020;
NOISE_HIGH  = 0.080;
TRAIN_FRAC  = 0.6;                     % training/validation split

na_vec = 2:6;                          % grid search ranges
nb_vec = 2:6;
nc_vec = 2:4;
```

---

## Troubleshooting

### `Connection refused` in MAVProxy
SITL crashed or didn't start. Kill everything and restart:
```bash
pkill -9 -f arducopter; pkill -9 -f mavproxy; pkill -9 -f sim_vehicle; pkill -9 -f gz
```

### `DetachableJoint should be attached to a model entity`
The DetachableJoint must be inside the iris model, not the world. Check:
```bash
grep -A3 "DetachableJoint" ~/ardupilot_gazebo/models/iris_with_ardupilot/model.sdf
```

### Drone tumbles immediately on takeoff
The launchpad is interfering with the rotors. The drone should spawn above any solid objects.

### MATLAB error: `Unrecognized table variable name 'drop_event'`
Old CSV format. Re-collect data with the latest `armax_run.py`.

### MATLAB error: `Dot indexing not supported`
Caused by multi-experiment iddata returning cell arrays. Always use the `safe_fit` helper at the bottom of `armax_v5.m`.

---

## Results Summary

Typical results from 5 runs of static-hover payload drop:

| Axis | Training Fit | Validation Fit | Notes |
|------|--------------|----------------|-------|
| Roll | ~92% | ~85% | Most excited axis - best identification |
| Pitch | ~95% | ~90% | Cleanest signal, weakest disturbance |
| Yaw | ~62% | ~55% | Barely excited - low fit is physically correct |

ARMAX model is stable (all poles inside unit circle), residuals pass Ljung-Box whiteness test, model degrades gracefully from clean to high-noise conditions.

---

## Future Work

- [ ] Figure-of-eight flight pattern with payload drop in motion
- [ ] Asymmetric (lopsided) payload for cross-axis coupling
- [ ] Compare against custom dropper drone on `main` branch
- [ ] RARMAX (recursive ARMAX) for online disturbance estimation
- [ ] Hardware validation on physical drone

---

## Citation / Authors
Repo: `https://github.com/erykpawelek/RARMAX_quadcopter_identyfiation`
Branch: `iris-drone-sim`
