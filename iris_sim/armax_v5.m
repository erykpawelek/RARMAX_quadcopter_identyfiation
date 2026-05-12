%% ================================================================
%% ARMAX Analysis - Iris Drone Payload Drop Experiment
%% Comprehensive identification with noise robustness study
%% ================================================================

clear; clc; close all;

%% ── Configuration ────────────────────────────────────────────
DATA_DIR    = '/home/philly/armax_data/';
Ts          = 0.01;                    % 100 Hz sampling
NOISE_LOW   = 0.005;                   % low noise sigma (deg)
NOISE_MED   = 0.020;                   % medium (typical IMU)
NOISE_HIGH  = 0.080;                   % high (cheap MEMS)
TRAIN_FRAC  = 0.6;
nk          = 1;

% Grid search ranges
na_vec = 2:6;
nb_vec = 2:6;
nc_vec = 2:4;

%% ── STEP 1: Auto-load CSVs ───────────────────────────────────
fprintf('=== STEP 1: Loading data ===\n');
files = dir(fullfile(DATA_DIR, 'run*_fig8_*.csv'));
if isempty(files)
    error('No run*.csv in %s', DATA_DIR);
end
N = length(files);
fprintf('  Found %d run files\n', N);

raw = cell(N, 1);
for i = 1:N
    fpath  = fullfile(files(i).folder, files(i).name);
    raw{i} = readtable(fpath, 'CommentStyle', '#');
    fprintf('  [%d] %s - %d samples\n', i, files(i).name, height(raw{i}));
end

%% ── STEP 2: Build iddata, detrend, split train/val ──────────
fprintf('\n=== STEP 2: Pre-processing ===\n');

ze_roll  = cell(N,1);
ze_pitch = cell(N,1);
ze_yaw   = cell(N,1);

for i = 1:N
    d = raw{i};
    u = d.drop_event;
    ze_roll{i}  = iddata(detrend(d.roll_deg),  u, Ts);
    ze_pitch{i} = iddata(detrend(d.pitch_deg), u, Ts);
    ze_yaw{i}   = iddata(detrend(d.yaw_deg),   u, Ts);
end

N_train = max(1, floor(TRAIN_FRAC * N));
N_val   = N - N_train;
fprintf('  Training runs:   %d\n', N_train);
fprintf('  Validation runs: %d\n', N_val);

ze_roll_train  = merge(ze_roll{1:N_train});
ze_pitch_train = merge(ze_pitch{1:N_train});
ze_yaw_train   = merge(ze_yaw{1:N_train});

if N_val > 0
    ze_roll_val  = merge(ze_roll{N_train+1:end});
    ze_pitch_val = merge(ze_pitch{N_train+1:end});
    ze_yaw_val   = merge(ze_yaw{N_train+1:end});
end

%% ── STEP 3: Grid search optimal orders (Roll axis) ──────────
fprintf('\n=== STEP 3: Order selection grid search ===\n');

n_a = numel(na_vec); n_b = numel(nb_vec); n_c = numel(nc_vec);
fit_grid  = nan(n_a, n_b, n_c);
aicc_grid = nan(n_a, n_b, n_c);
mse_grid  = nan(n_a, n_b, n_c);

best_fit    = -Inf;
best_orders = [4 4 4 1];
best_model  = [];

for ia = 1:n_a
    for ib = 1:n_b
        for ic = 1:n_c
            try
                m = armax(ze_roll_train, ...
                    [na_vec(ia) nb_vec(ib) nc_vec(ic) nk]);

                fp = safe_fit(m);

                fit_grid(ia,ib,ic)  = fp;
                aicc_grid(ia,ib,ic) = aic(m, 'AICc');
                mse_grid(ia,ib,ic)  = mean(m.Report.Fit.MSE);

                if fp > best_fit
                    best_fit    = fp;
                    best_orders = [na_vec(ia) nb_vec(ib) nc_vec(ic) nk];
                    best_model  = m;
                end
            catch
            end
        end
    end
end

% Fallback if grid search failed
if isempty(best_model)
    fprintf('  Grid search failed, using default ARMAX[4 4 4 1]\n');
    best_orders = [4 4 4 1];
    best_model  = armax(ze_roll_train, best_orders);
    best_fit    = safe_fit(best_model);
end

fprintf('  Best orders: ARMAX[%d %d %d %d]\n', best_orders);
fprintf('  Best fit:    %.2f%%\n', best_fit);

%% ── STEP 4: Fit ARMAX on all 3 axes ──────────────────────────
fprintf('\n=== STEP 4: Fitting Roll, Pitch, Yaw ===\n');
model_roll  = best_model;
model_pitch = armax(ze_pitch_train, best_orders);
model_yaw   = armax(ze_yaw_train,   best_orders);

fit_roll  = safe_fit(model_roll);
fit_pitch = safe_fit(model_pitch);
fit_yaw   = safe_fit(model_yaw);

fprintf('  Roll  fit: %.2f%%\n', fit_roll);
fprintf('  Pitch fit: %.2f%%\n', fit_pitch);
fprintf('  Yaw   fit: %.2f%%\n', fit_yaw);

%% ── STEP 5: ARX comparison ──────────────────────────────────
fprintf('\n=== STEP 5: ARX vs ARMAX comparison ===\n');
arx_roll = arx(ze_roll_train, [best_orders(1) best_orders(2) nk]);
fit_arx_roll = safe_fit(arx_roll);
fprintf('  Roll ARX  fit: %.2f%%\n', fit_arx_roll);
fprintf('  Roll ARMAX fit: %.2f%% (improvement: %.2f%%)\n', ...
    fit_roll, fit_roll - fit_arx_roll);

%% ── STEP 6: Cross-validation ────────────────────────────────
if N_val > 0
    fprintf('\n=== STEP 6: Cross-validation ===\n');
    [~, fit_val_roll]  = compare(ze_roll_val,  model_roll);
    [~, fit_val_pitch] = compare(ze_pitch_val, model_pitch);
    [~, fit_val_yaw]   = compare(ze_yaw_val,   model_yaw);

    if iscell(fit_val_roll);  fit_val_roll  = mean(cell2mat(fit_val_roll));  end
    if iscell(fit_val_pitch); fit_val_pitch = mean(cell2mat(fit_val_pitch)); end
    if iscell(fit_val_yaw);   fit_val_yaw   = mean(cell2mat(fit_val_yaw));   end

    fprintf('  Roll  validation: %.2f%%\n', fit_val_roll);
    fprintf('  Pitch validation: %.2f%%\n', fit_val_pitch);
    fprintf('  Yaw   validation: %.2f%%\n', fit_val_yaw);
end

%% ── STEP 7: Noise robustness with 3 levels ──────────────────
fprintf('\n=== STEP 7: Noise robustness (3 levels) ===\n');

noise_levels = [NOISE_LOW, NOISE_MED, NOISE_HIGH];
noise_names  = {'Low', 'Medium', 'High'};
n_levels     = length(noise_levels);

fit_noisy_roll   = zeros(1, n_levels);
fit_noisy_pitch  = zeros(1, n_levels);
fit_noisy_yaw    = zeros(1, n_levels);
models_roll_noisy = cell(1, n_levels);

rng(42);

for k = 1:n_levels
    sigma = noise_levels(k);

    ze_r_n = cell(N_train, 1);
    ze_p_n = cell(N_train, 1);
    ze_y_n = cell(N_train, 1);
    for i = 1:N_train
        n_r = sigma * randn(size(ze_roll{i}.OutputData));
        n_p = sigma * randn(size(ze_pitch{i}.OutputData));
        n_y = sigma * randn(size(ze_yaw{i}.OutputData));
        ze_r_n{i} = iddata(ze_roll{i}.OutputData  + n_r, ze_roll{i}.InputData,  Ts);
        ze_p_n{i} = iddata(ze_pitch{i}.OutputData + n_p, ze_pitch{i}.InputData, Ts);
        ze_y_n{i} = iddata(ze_yaw{i}.OutputData   + n_y, ze_yaw{i}.InputData,   Ts);
    end

    m_r = armax(merge(ze_r_n{:}), best_orders);
    m_p = armax(merge(ze_p_n{:}), best_orders);
    m_y = armax(merge(ze_y_n{:}), best_orders);

    fit_noisy_roll(k)  = safe_fit(m_r);
    fit_noisy_pitch(k) = safe_fit(m_p);
    fit_noisy_yaw(k)   = safe_fit(m_y);
    models_roll_noisy{k} = m_r;

    fprintf('  %s noise (sigma=%.3f deg):\n', noise_names{k}, sigma);
    fprintf('    Roll:  %.2f%%  (clean: %.2f%%, drop: %.2f%%)\n', ...
        fit_noisy_roll(k), fit_roll, fit_roll - fit_noisy_roll(k));
    fprintf('    Pitch: %.2f%%  (clean: %.2f%%, drop: %.2f%%)\n', ...
        fit_noisy_pitch(k), fit_pitch, fit_pitch - fit_noisy_pitch(k));
    fprintf('    Yaw:   %.2f%%  (clean: %.2f%%, drop: %.2f%%)\n', ...
        fit_noisy_yaw(k), fit_yaw, fit_yaw - fit_noisy_yaw(k));
end

%% ── STEP 8: Pole-zero stability ─────────────────────────────
fprintf('\n=== STEP 8: Pole-zero stability ===\n');
poles_roll  = roots(model_roll.A);
max_pole_radius = max(abs(poles_roll));

fprintf('  Roll model: max |pole| = %.4f\n', max_pole_radius);
if max_pole_radius < 1
    fprintf('  STABLE - all poles inside unit circle\n');
else
    fprintf('  UNSTABLE - pole outside unit circle!\n');
end

%% ── STEP 9: Residual whiteness (Ljung-Box) ──────────────────
fprintf('\n=== STEP 9: Residual whiteness test ===\n');
try
    res = pe(model_roll, ze_roll_train);
    res_data = res.OutputData;
    if iscell(res_data); res_data = cell2mat(res_data); end
    [h_lb, p_lb] = lbqtest(res_data, 'Lags', 20);
    if h_lb == 0
        fprintf('  PASS - residuals are white (p = %.4f > 0.05)\n', p_lb);
    else
        fprintf('  FAIL - residuals not white (p = %.4f)\n', p_lb);
    end
catch
    fprintf('  Whiteness test skipped (Statistics Toolbox required)\n');
    h_lb = -1; p_lb = NaN;
end

%% ────────────────────────────────────────────────────────────
%% FIGURES
%% ────────────────────────────────────────────────────────────

%% Figure 1: Grid search heatmaps
figure('Name', 'Grid Search', 'Color', 'w', 'Position', [50 50 1300 350]);
fit_2d  = max(fit_grid, [], 3);
mse_2d  = min(mse_grid, [], 3);
aicc_2d = min(aicc_grid, [], 3);

subplot(1,3,1); imagesc(nb_vec, na_vec, fit_2d); colorbar;
xlabel('nb'); ylabel('na'); title('Fit % (best over nc)'); axis square;

subplot(1,3,2); imagesc(nb_vec, na_vec, mse_2d); colorbar;
xlabel('nb'); ylabel('na'); title('MSE'); axis square;

subplot(1,3,3); imagesc(nb_vec, na_vec, aicc_2d); colorbar;
xlabel('nb'); ylabel('na'); title('AICc'); axis square;

sgtitle(sprintf('Grid Search - Roll Axis (best: ARMAX[%d %d %d %d])', best_orders));

%% Figure 2: Time domain fit
figure('Name', 'ARMAX Fit', 'Color', 'w', 'Position', [100 100 1200 800]);
subplot(3,1,1); compare(ze_roll_train, model_roll);
title(sprintf('Roll  ARMAX[%d %d %d %d] - Fit: %.2f%%', best_orders, fit_roll));
ylabel('Roll (deg)'); grid on;

subplot(3,1,2); compare(ze_pitch_train, model_pitch);
title(sprintf('Pitch - Fit: %.2f%%', fit_pitch));
ylabel('Pitch (deg)'); grid on;

subplot(3,1,3); compare(ze_yaw_train, model_yaw);
title(sprintf('Yaw - Fit: %.2f%%', fit_yaw));
ylabel('Yaw (deg)'); xlabel('Time (s)'); grid on;
sgtitle('ARMAX Model vs Measured Data');

%% Figure 3: Residuals
figure('Name', 'Residuals', 'Color', 'w', 'Position', [150 150 1200 800]);
subplot(3,1,1); resid(ze_roll_train,  model_roll);  title('Roll');  grid on;
subplot(3,1,2); resid(ze_pitch_train, model_pitch); title('Pitch'); grid on;
subplot(3,1,3); resid(ze_yaw_train,   model_yaw);   title('Yaw');   grid on;
sgtitle('Residual Analysis');

%% Figure 4: Noise robustness
figure('Name', 'Noise Robustness', 'Color', 'w', 'Position', [200 200 1200 700]);

bar_data = [fit_roll,  fit_pitch,  fit_yaw;
            fit_noisy_roll(1), fit_noisy_pitch(1), fit_noisy_yaw(1);
            fit_noisy_roll(2), fit_noisy_pitch(2), fit_noisy_yaw(2);
            fit_noisy_roll(3), fit_noisy_pitch(3), fit_noisy_yaw(3)];

subplot(2,1,1);
bar(bar_data, 'grouped');
set(gca, 'XTickLabel', {'Clean', ...
    sprintf('Low (\\sigma=%.3f)', NOISE_LOW), ...
    sprintf('Med (\\sigma=%.3f)', NOISE_MED), ...
    sprintf('High (\\sigma=%.3f)', NOISE_HIGH)});
ylabel('Fit %'); ylim([0 100]); grid on;
legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');
title('Identification Fit % vs Noise Level');

subplot(2,1,2);
deg_roll  = fit_roll  - fit_noisy_roll;
deg_pitch = fit_pitch - fit_noisy_pitch;
deg_yaw   = fit_yaw   - fit_noisy_yaw;
bar([deg_roll; deg_pitch; deg_yaw]', 'grouped');
set(gca, 'XTickLabel', noise_names);
ylabel('Fit drop (%)'); xlabel('Noise level');
legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');
title('Performance Degradation Due to Noise');
grid on;

sgtitle('Noise Robustness Analysis');

%% Figure 5: Bode comparison
figure('Name', 'Bode Response', 'Color', 'w', 'Position', [250 250 950 600]);
bode(model_roll, models_roll_noisy{1}, models_roll_noisy{2}, models_roll_noisy{3});
legend('Clean', sprintf('Low \\sigma=%.3f', NOISE_LOW), ...
       sprintf('Med \\sigma=%.3f', NOISE_MED), ...
       sprintf('High \\sigma=%.3f', NOISE_HIGH), 'Location', 'best');
title('Frequency Response - Roll Dynamics across Noise Levels');
grid on;

%% Figure 6: Pole-zero map
figure('Name', 'Pole-Zero', 'Color', 'w', 'Position', [300 300 700 700]);
pzmap(model_roll);
title('Pole-Zero Map - Roll ARMAX Model');
grid on; axis equal;

%% Figure 7: Raw data overview
figure('Name', 'Raw Data', 'Color', 'w', 'Position', [350 350 1200 800]);
colors = lines(N);

subplot(3,1,1); hold on; grid on;
for i = 1:N; plot(raw{i}.time_s, raw{i}.roll_deg, 'Color', colors(i,:)); end
title('Roll - All Runs'); ylabel('Degrees');
legend(arrayfun(@(k) sprintf('Run %d', k), 1:N, 'UniformOutput', false));

subplot(3,1,2); hold on; grid on;
for i = 1:N; plot(raw{i}.time_s, raw{i}.pitch_deg, 'Color', colors(i,:)); end
title('Pitch - All Runs'); ylabel('Degrees');

subplot(3,1,3); hold on; grid on;
for i = 1:N; plot(raw{i}.time_s, raw{i}.yaw_deg, 'Color', colors(i,:)); end
title('Yaw - All Runs'); ylabel('Degrees'); xlabel('Time (s)');

sgtitle('Raw IMU Data - All Runs');

%% ── SUMMARY ─────────────────────────────────────────────────
fprintf('\n');
fprintf('=================================================\n');
fprintf('          ARMAX ANALYSIS SUMMARY\n');
fprintf('=================================================\n');
fprintf('  Runs analyzed:       %d (train: %d, val: %d)\n', N, N_train, N_val);
fprintf('  Best orders:         ARMAX[%d %d %d %d]\n', best_orders);
fprintf('  Sample time Ts:      %.3f s\n', Ts);
fprintf('-------------------------------------------------\n');
fprintf('  TRAINING FIT:\n');
fprintf('    Roll:  %.2f %%\n', fit_roll);
fprintf('    Pitch: %.2f %%\n', fit_pitch);
fprintf('    Yaw:   %.2f %%\n', fit_yaw);
if N_val > 0
    fprintf('-------------------------------------------------\n');
    fprintf('  VALIDATION FIT:\n');
    fprintf('    Roll:  %.2f %%\n', fit_val_roll);
    fprintf('    Pitch: %.2f %%\n', fit_val_pitch);
    fprintf('    Yaw:   %.2f %%\n', fit_val_yaw);
end
fprintf('-------------------------------------------------\n');
fprintf('  ARX vs ARMAX (Roll): +%.2f %% improvement\n', fit_roll - fit_arx_roll);
fprintf('-------------------------------------------------\n');
fprintf('  NOISE ROBUSTNESS (Roll):\n');
fprintf('    Clean:           %.2f %%\n', fit_roll);
fprintf('    Low  (sigma=%.3f): %.2f %%\n', NOISE_LOW,  fit_noisy_roll(1));
fprintf('    Med  (sigma=%.3f): %.2f %%\n', NOISE_MED,  fit_noisy_roll(2));
fprintf('    High (sigma=%.3f): %.2f %%\n', NOISE_HIGH, fit_noisy_roll(3));
fprintf('-------------------------------------------------\n');
if max_pole_radius < 1
    fprintf('  Stability:           STABLE (max pole radius %.4f)\n', max_pole_radius);
else
    fprintf('  Stability:           UNSTABLE (max pole radius %.4f)\n', max_pole_radius);
end
if h_lb == 0
    fprintf('  Whiteness test:      PASS (p = %.4f)\n', p_lb);
elseif h_lb == -1
    fprintf('  Whiteness test:      SKIPPED\n');
else
    fprintf('  Whiteness test:      FAIL (p = %.4f)\n', p_lb);
end
fprintf('=================================================\n');

% Save models
save('armax_models.mat', 'model_roll', 'model_pitch', 'model_yaw', ...
     'models_roll_noisy', 'best_orders', 'Ts', 'noise_levels');
fprintf('\nModels saved to armax_models.mat\n');

%% ── HELPER FUNCTION ─────────────────────────────────────────
function fp = safe_fit(m)
    fp = m.Report.Fit.FitPercent;
    if iscell(fp)
        fp = cell2mat(fp);
    end
    if isempty(fp)
        fp = 0;
    else
        fp = mean(fp(~isnan(fp)));
        if isempty(fp) || isnan(fp)
            fp = 0;
        end
    end
end