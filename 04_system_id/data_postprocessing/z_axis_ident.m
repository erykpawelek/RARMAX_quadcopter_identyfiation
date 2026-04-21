clc 
clear all 

load('z_axis_ident.mat')
markers.time(4) = markers.time(4) + 0.94;

vib_hover_filt_20 = load('vib_hover_filt_20.mat');
vib_hover_filt_20.markers.time(3) = vib_hover_filt_20.markers.time(3) + 0.94;

%% Extracting Hover Operating Points
% Extracting marker strings for 20Hz data
marker_str_20 = string(vib_hover_filt_20.markers.text);

% Finding indices of specific markers
idx_hp_start_20 = find(contains(marker_str_20, 'HOVER_PAYLOAD_START'));
idx_hp_end_20   = find(contains(marker_str_20, 'HOVER_PAYLOAD_STOP'));
idx_hnp_start_20 = find(contains(marker_str_20, 'HOVER_NO_PAYLOAD_START'));
idx_hnp_end_20   = find(contains(marker_str_20, 'HOVER_NO_PAYLOAD_STOP'));

% Extracting exact time values for markers
t_hp_start_20 = vib_hover_filt_20.markers.time(idx_hp_start_20);
t_hp_end_20   = vib_hover_filt_20.markers.time(idx_hp_end_20);
t_hnp_start_20 = vib_hover_filt_20.markers.time(idx_hnp_start_20);
t_hnp_end_20   = vib_hover_filt_20.markers.time(idx_hnp_end_20);

% CRITICAL FIX: Create logical masks using RCOU time vector, not IMU time
mask_rcou_hp_20 = (vib_hover_filt_20.rcou.time >= t_hp_start_20) & (vib_hover_filt_20.rcou.time <= t_hp_end_20);
mask_rcou_hnp_20 = (vib_hover_filt_20.rcou.time >= t_hnp_start_20) & (vib_hover_filt_20.rcou.time <= t_hnp_end_20);

% Calculating mean values of PWM during hover for system identification
C1_payload = vib_hover_filt_20.rcou.c1(mask_rcou_hp_20);
C2_payload = vib_hover_filt_20.rcou.c2(mask_rcou_hp_20);
C3_payload = vib_hover_filt_20.rcou.c3(mask_rcou_hp_20);
C4_payload = vib_hover_filt_20.rcou.c4(mask_rcou_hp_20);
u_mean_payload_hover = mean( (double(C1_payload) + double(C2_payload) + double(C3_payload) + double(C4_payload)) / 4 );

C1_no_payload = vib_hover_filt_20.rcou.c1(mask_rcou_hnp_20);
C2_no_payload = vib_hover_filt_20.rcou.c2(mask_rcou_hnp_20);
C3_no_payload = vib_hover_filt_20.rcou.c3(mask_rcou_hnp_20);
C4_no_payload = vib_hover_filt_20.rcou.c4(mask_rcou_hnp_20);
u_mean_no_payload_hover = mean( (double(C1_no_payload) + double(C2_no_payload) + double(C3_no_payload) + double(C4_no_payload)) / 4 );

disp(['Imported PWM Operating Point (Attached): ', num2str(u_mean_payload_hover), ' us']);
disp(['Imported PWM Operating Point (Detached): ', num2str(u_mean_no_payload_hover), ' us']);

%% Accelerations plots and PWM plots
figure('Name','Acceleration values along X,Y,Z axis');
subplot(2,1,1);
plot(imu.time, imu.accX, '-r', imu.time, imu.accY, '--b'); xlim([50 120])
xlabel('Time [s]');ylabel('Acceleration [m/s^2]')
grid on;
for i = 1:length(markers.time)
    m_time = markers.time(i);
    m_text = string(markers.text(i,:)); 
    xline(m_time, '--m');   
end
legend('Acceleration X', 'Acceleration Y');

subplot(2,1,2);
plot(imu.time, imu.accZ, '-g'); xlim([50 120])
xlabel('Time [s]');ylabel('Acceleration [m/s^2]')
grid on;
for i = 1:length(markers.time)
    m_time = markers.time(i);
    m_text = string(markers.text(i,:)); 
    xline(m_time, '--m');   
end
legend('Acceleration Z');
sgtitle('Acceleration plots during whole identification sequence');

figure('Name', 'Motor PWM Signals during whole identification sequence');
u_mean_raw_plot = (double(rcou.c1) + double(rcou.c2) + double(rcou.c3) + double(rcou.c4)) / 4;
plot(rcou.time, u_mean_raw_plot, 'g', 'LineWidth', 1.2); 
xlim([50 120]); 
xlabel('Time [s]'); 
ylabel('Average PWM [\mus]');
title('Average Motor PWM Signal with Event Markers');
grid on;
hold on;
for i = 1:length(markers.time)
    m_time = markers.time(i);
    xline(m_time, '--m', 'LineWidth', 1.5);   
end
legend('Average Motor PWM');

%% System identification - Data Preparation & ARMAX Formatting

% Extracting marker times
marker_str = string(markers.text);
idx_noise1_start = find(contains(marker_str, 'NOISE_1_START'));
idx_noise1_end   = find(contains(marker_str, 'NOISE_1_STOP'));
idx_noise2_start = find(contains(marker_str, 'NOISE_2_START'));
idx_noise2_end   = find(contains(marker_str, 'NOISE_2_STOP'));

t_noise1_start = markers.time(idx_noise1_start);
t_noise1_end   = markers.time(idx_noise1_end);
t_noise2_start = markers.time(idx_noise2_start);
t_noise2_end   = markers.time(idx_noise2_end);

% Mathematical time grid for ARMAX Ts = 0.02s
Fs_armax = 50;
Ts = 1 / Fs_armax;
t_grid_noise1 = t_noise1_start : Ts : t_noise1_end;
t_grid_noise2 = t_noise2_start : Ts : t_noise2_end;

% Calculate Effective PWM for the entire log first
u_mean_raw = (double(rcou.c1) + double(rcou.c2) + double(rcou.c3) + double(rcou.c4)) / 4;
roll_rad_all = deg2rad(interp1(att.time, att.roll, rcou.time, 'linear'));
pitch_rad_all = deg2rad(interp1(att.time, att.pitch, rcou.time, 'linear'));
u_eff_all = u_mean_raw .* cos(roll_rad_all) .* cos(pitch_rad_all);

% =========================================================
% SETTING TRUE OPERATING POINTS
% =========================================================
% Assigning PWM baselines calculated from the hover log
u_eff_op_n1 = u_mean_payload_hover; 
u_eff_op_n2 = u_mean_no_payload_hover; 

% Finding local altitude equilibrium just before noise injection (3s window)
t_hover1_start = t_noise1_start - 3.0;
t_hover1_end   = t_noise1_start - 0.5;
t_hover2_start = t_noise2_start - 3.0;
t_hover2_end   = t_noise2_start - 0.5;

mask_hover1_ctun = (ctun.time >= t_hover1_start) & (ctun.time <= t_hover1_end);
alt_op_n1 = mean(ctun.alt(mask_hover1_ctun));

mask_hover2_ctun = (ctun.time >= t_hover2_start) & (ctun.time <= t_hover2_end);
alt_op_n2 = mean(ctun.alt(mask_hover2_ctun));

% =========================================================
% PROCESSING NOISE 1 (Payload Attached) 
% =========================================================
% Extract and interpolate output: Altitude
y_alt_n1 = interp1(ctun.time, ctun.alt, t_grid_noise1, 'linear')';
% Detrend using stable altitude baseline
y_alt_n1_detrend = y_alt_n1 - alt_op_n1;

% Extract and interpolate input: Motor PWM
u_eff_n1 = interp1(rcou.time, u_eff_all, t_grid_noise1, 'linear')';
% Detrend using imported PWM baseline
u_eff_n1_detrend = u_eff_n1 - u_eff_op_n1;

data_noise1 = iddata(y_alt_n1_detrend, u_eff_n1_detrend, Ts, ...
    'InputName', 'Effective PWM Deviation', 'OutputName', 'Altitude Deviation');

% =========================================================
% PROCESSING NOISE 2 (Payload Detached)
% =========================================================
% Extract and interpolate output: Altitude
y_alt_n2 = interp1(ctun.time, ctun.alt, t_grid_noise2, 'linear')';
y_alt_n2_detrend = y_alt_n2 - alt_op_n2;

% Extract and interpolate input: Motor PWM
u_eff_n2 = interp1(rcou.time, u_eff_all, t_grid_noise2, 'linear')';
u_eff_n2_detrend = u_eff_n2 - u_eff_op_n2;

% Create iddata object
data_noise2 = iddata(y_alt_n2_detrend, u_eff_n2_detrend, Ts, ...
    'InputName', 'Effective PWM Deviation', 'OutputName', 'Altitude Deviation');

% PLOTTING PREPARED DATA
figure('Name', 'Prepared Data Overview (True Baselines)');
subplot(2,1,1);
plot(data_noise1);
title('Prepared ARMAX Data - Noise 1 (Payload Attached)');
subplot(2,1,2);
plot(data_noise2);
title('Prepared ARMAX Data - Noise 2 (Payload Detached)');

% VISUALIZATION: RAW VS INTERPOLATED DATA 
figure('Name', 'Interpolation Verification');
plot(t_grid_noise1, y_alt_n1, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Interpolated Grid (Strict 50Hz)');hold on;
grid on;
plot(ctun.time, ctun.alt, 'g.', 'MarkerSize', 8, 'DisplayName', 'Raw Log Data (Variable Step)');
xlim([t_noise1_start, t_noise1_end]);
title('Interpolation Verification - Full Altitude Waveform');
xlabel('Time [s]');
ylabel('Altitude [m]');
legend('Location', 'best');

figure('Name', 'Interpolation Verification - PWM Input');
% Plot the 50Hz interpolated PWM
plot(t_grid_noise1, u_eff_n1, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Interpolated PWM (Strict 50Hz)'); hold on;
grid on;
% Plot the raw, variable-step PWM from logs
plot(rcou.time, u_eff_all, 'g.', 'MarkerSize', 8, 'DisplayName', 'Raw PWM Log Data');

% CRITICAL: We zoom in on a small 2-second window. 
% If you look at the whole 20s, it's just a thick block of color.
xlim([t_noise1_start + 5, t_noise1_start + 7]); 
title('Interpolation Verification - Effective PWM (Zoomed 2s window)');
xlabel('Time [s]');
ylabel('Effective PWM [\mus]');
legend('Location', 'best');

%% ARMAX MODEL ESTIMATION - GRID SEARCH (PHYSICS CONSTRAINED)
disp('Starting ARMAX grid search optimized for PURE SIMULATION...');

% Search for Noise 1 (Payload Attached) 
best_fit_n1 = -Inf;
best_sys_n1 = [];
best_orders_n1 = [1 1 1 1];
disp('Searching optimal configuration for Payload Attached...');

% Orders limited to 4 to prevent overfitting to SITL noise
for na = 1:4 
    for nb = 1:4
        for nc = 1:4
            for nk = 1:2 
                current_orders = [na nb nc nk];
                try
                    % Estimate model quietly
                    opt = armaxOptions('Display', 'off'); 
                    temp_sys = armax(data_noise1, current_orders, opt);
                    
                    % Evaluate fitness using Pure Simulation
                    [~, current_fit] = compare(data_noise1, temp_sys);
                    
                    % Update variables if current model is better
                    if current_fit > best_fit_n1
                        best_fit_n1 = current_fit;
                        best_sys_n1 = temp_sys;
                        best_orders_n1 = current_orders;
                    end
                catch
                    continue;
                end
            end
        end
    end
end
disp(['Best orders for Payload Attached [na nb nc nk]: ', num2str(best_orders_n1)]);
disp(['Achieved SIMULATION Fit: ', num2str(best_fit_n1), '%']);

% Search for Noise 2 (Payload Detached) 
best_fit_n2 = -Inf;
best_sys_n2 = [];
best_orders_n2 = [1 1 1 1];
disp('Searching optimal configuration for Payload Detached...');

for na = 1:4
    for nb = 1:4
        for nc = 1:4
            for nk = 1:2
                current_orders = [na nb nc nk];
                try
                    opt = armaxOptions('Display', 'off'); 
                    temp_sys = armax(data_noise2, current_orders, opt);
                   
                    [~, current_fit] = compare(data_noise2, temp_sys);
                    
                    if current_fit > best_fit_n2
                        best_fit_n2 = current_fit;
                        best_sys_n2 = temp_sys;
                        best_orders_n2 = current_orders;
                    end
                catch
                    continue;
                end
            end
        end
    end
end
disp(['Best orders for Payload Detached [na nb nc nk]: ', num2str(best_orders_n2)]);
disp(['Achieved SIMULATION Fit: ', num2str(best_fit_n2), '%']);

%% MODEL VALIDATION WITH OPTIMAL MODELS
% Model Fit Plots
figure('Name', 'ARMAX Model Validation (Optimal Configurations)');
subplot(2,1,1);
compare(data_noise1, best_sys_n1);
title(['Simulation Fit - Noise 1 (Payload Attached) | Orders: ', num2str(best_orders_n1)]);
subplot(2,1,2);
compare(data_noise2, best_sys_n2);
title(['Simulation Fit - Noise 2 (Payload Detached) | Orders: ', num2str(best_orders_n2)]);

% RESIDUAL ANALYSIS
figure('Name', 'Residual Analysis - Optimal Noise 1');
resid(data_noise1, best_sys_n1);
title('Residual Analysis - Optimal Noise 1 (Payload Attached)');

figure('Name', 'Residual Analysis - Optimal Noise 2');
resid(data_noise2, best_sys_n2);
title('Residual Analysis - Optimal Noise 2 (Payload Detached)');

%% Frequency Response Analysis (Bode Plots)
disp('Calculating empirical frequency response using spectral analysis...');

% Estimate empirical transfer functions using spectral analysis (spa)
sys_empirical_n1 = spa(data_noise1);
sys_empirical_n2 = spa(data_noise2);

% Define Bode plot options
opts = bodeoptions;
opts.FreqUnits = 'Hz'; 
opts.Grid = 'on';

% Plot 1: Payload Attached (Empirical vs ARMAX)
figure('Name', 'System Frequency Response (Payload Attached)');
bode(sys_empirical_n1, 'b', best_sys_n1, 'r--', opts);
title(['Bode Plot Comparison: Noise 1 (Payload Attached) | ARMAX: ', num2str(best_orders_n1)]);
legend('Empirical Data (Raw Signal)', 'ARMAX Model', 'Location', 'southwest');

% Plot 2: Payload Detached (Empirical vs ARMAX)
figure('Name', 'System Frequency Response (Payload Detached)');
bode(sys_empirical_n2, 'b', best_sys_n2, 'r--', opts);
title(['Bode Plot Comparison: Noise 2 (Payload Detached) | ARMAX: ', num2str(best_orders_n2)]);
legend('Empirical Data (Raw Signal)', 'ARMAX Model', 'Location', 'southwest');

%% Flight Simulation - Step Response
figure('Name', 'Step Response: Altitude tracking simulation');
% Simulating a sudden +50 microseconds increase in effective PWM
opt = stepDataOptions('StepAmplitude', 50);
step(best_sys_n1, 'r', best_sys_n2, 'b', 3, opt);
grid on;
title('Simulated Flight Command: Reaction to sudden +50\mus PWM step');
xlabel('Time [s]');
ylabel('Altitude Change [m]');
legend('Noise 1 (Payload Attached)', 'Noise 2 (Payload Detached)', 'Location', 'northwest');

%% CROSS-VALIDATION WITH REAL FLIGHT DATA
disp('Starting validation sequence using external flight log...');
% Load the validation data
validation_data = load('z_validation.mat');

% Extract marker timings from the validation log
val_marker_str = string(validation_data.markers.text);
idx_val_p_start  = find(contains(val_marker_str, 'START_STEP_PAYLOAD'));
idx_val_p_stop   = find(contains(val_marker_str, 'STOP_STEP_PAYLOAD'));
idx_val_np_start = find(contains(val_marker_str, 'START_STEP_NO_PAYLOAD'));
idx_val_np_stop  = find(contains(val_marker_str, 'STOP_STEP_NO_PAYLOAD'));

% Get exact timestamps for the stimuli phases
t_val_p_start  = validation_data.markers.time(idx_val_p_start);
t_val_p_stop   = validation_data.markers.time(idx_val_p_stop);
t_val_np_start = validation_data.markers.time(idx_val_np_start);
t_val_np_stop  = validation_data.markers.time(idx_val_np_stop);

% Create uniform time grids for linear simulation (Ts = 0.02s / 50Hz)
t_grid_val_p  = t_val_p_start : Ts : t_val_p_stop;
t_grid_val_np = t_val_np_start : Ts : t_val_np_stop;

% Calculate Effective PWM for the entire validation log (eliminating tilt/drift)
u_mean_raw_val = (double(validation_data.rcou.c1) + double(validation_data.rcou.c2) + ...
                  double(validation_data.rcou.c3) + double(validation_data.rcou.c4)) / 4;
% Interpolate attitudes to match RCOU timestamps and convert to radians
roll_rad_val  = deg2rad(interp1(validation_data.att.time, validation_data.att.roll, validation_data.rcou.time, 'linear'));
pitch_rad_val = deg2rad(interp1(validation_data.att.time, validation_data.att.pitch, validation_data.rcou.time, 'linear'));
% Calculate the pure vertical thrust component
u_eff_all_val = u_mean_raw_val .* cos(roll_rad_val) .* cos(pitch_rad_val);

% =========================================================
% VALIDATION PHASE 1: Payload Attached
% =========================================================
% 1. Prepare and detrend the input (Effective PWM)
u_eff_val_p = interp1(validation_data.rcou.time, u_eff_all_val, t_grid_val_p, 'linear')';
% CRITICAL: Using the exact same operating point found during identification
u_eff_val_p_detrend = u_eff_val_p - u_mean_payload_hover;

% 2. Prepare and detrend the output (Altitude)
y_alt_val_p = interp1(validation_data.ctun.time, validation_data.ctun.alt, t_grid_val_p, 'linear')';
% Find baseline altitude right before the step (2-second window)
mask_hover_val_p = (validation_data.ctun.time >= (t_val_p_start - 2.0)) & (validation_data.ctun.time <= t_val_p_start);
alt_op_val_p = mean(validation_data.ctun.alt(mask_hover_val_p));
y_alt_val_p_detrend = y_alt_val_p - alt_op_val_p;

% 3. Simulate ARMAX model response to the actual recorded PWM step
% lsim requires a time vector starting from 0
t_lsim_p = t_grid_val_p - t_grid_val_p(1);
[y_sim_p, ~] = lsim(best_sys_n1, u_eff_val_p_detrend, t_lsim_p);

% =========================================================
% VALIDATION PHASE 2: Payload Detached
% =========================================================
% 1. Prepare and detrend the input (Effective PWM)
u_eff_val_np = interp1(validation_data.rcou.time, u_eff_all_val, t_grid_val_np, 'linear')';
% CRITICAL: Using the exact same operating point found during identification
u_eff_val_np_detrend = u_eff_val_np - u_mean_no_payload_hover;

% 2. Prepare and detrend the output (Altitude)
y_alt_val_np = interp1(validation_data.ctun.time, validation_data.ctun.alt, t_grid_val_np, 'linear')';
% Find baseline altitude right before the step (2-second window)
mask_hover_val_np = (validation_data.ctun.time >= (t_val_np_start - 2.0)) & (validation_data.ctun.time <= t_val_np_start);
alt_op_val_np = mean(validation_data.ctun.alt(mask_hover_val_np));
y_alt_val_np_detrend = y_alt_val_np - alt_op_val_np;

% 3. Simulate ARMAX model response to the actual recorded PWM step
t_lsim_np = t_grid_val_np - t_grid_val_np(1);
[y_sim_np, ~] = lsim(best_sys_n2, u_eff_val_np_detrend, t_lsim_np);

% =========================================================
% PLOTTING RESULTS
% =========================================================
figure('Name', 'ARMAX Cross-Validation with Real Flight Logs');

% Plot 1: Payload Attached Comparison
subplot(2,1,1);
plot(t_lsim_p, y_alt_val_p_detrend, 'g-', 'LineWidth', 2, 'DisplayName', 'Real Flight Log Data'); hold on;
plot(t_lsim_p, y_sim_p, 'r--', 'LineWidth', 2, 'DisplayName', ['Simulated ARMAX ', num2str(best_orders_n1)]);
grid on;
title('Model Validation: Response to real PWM step (Payload Attached)');
xlabel('Time [s]');
ylabel('Altitude Change [m]');
legend('Location', 'northwest');

% Plot 2: Payload Detached Comparison
subplot(2,1,2);
plot(t_lsim_np, y_alt_val_np_detrend, 'g-', 'LineWidth', 2, 'DisplayName', 'Real Flight Log Data'); hold on;
plot(t_lsim_np, y_sim_np, 'b--', 'LineWidth', 2, 'DisplayName', ['Simulated ARMAX ', num2str(best_orders_n2)]);
grid on;
title('Model Validation: Response to real PWM step (Payload Detached)');
xlabel('Time [s]');
ylabel('Altitude Change [m]');
legend('Location', 'northwest');

disp('Validation complete. Compare the solid black line (reality) with dashed lines (simulation).');