%% ================================================================
%% ARMAX Comparison: Symmetric vs Lopsided Payload
%% ================================================================
%% Loads two datasets (symmetric and asymmetric payload drops),
%% fits ARMAX models on each, and produces side-by-side comparison.
%%
%% Hypothesis: Lopsided payload creates multi-axis disturbance
%% (cross-coupling between roll/pitch/yaw), making identification
%% harder than the cleanly-isolated symmetric drop.
%% ================================================================

clear; clc; close all;

%% ── Configuration ────────────────────────────────────────────
DIR_SYM    = '/home/philly/armax_data/';           % symmetric runs
DIR_LOP    = '/home/philly/armax_data_lopsided/';  % lopsided runs
Ts         = 0.01;                                  % 100 Hz
ORDERS     = [4 4 4 1];                             % ARMAX[na nb nc nk]
TRAIN_FRAC = 0.6;

%% ── Load both datasets ──────────────────────────────────────
fprintf('=== Loading datasets ===\n');

[ze_roll_s, ze_pitch_s, ze_yaw_s, raw_s, N_s] = load_runs(DIR_SYM, Ts);
fprintf('  Symmetric: %d runs loaded\n', N_s);

[ze_roll_l, ze_pitch_l, ze_yaw_l, raw_l, N_l] = load_runs(DIR_LOP, Ts);
fprintf('  Lopsided:  %d runs loaded\n', N_l);

%% ── Train/validation split ──────────────────────────────────
N_train_s = max(1, floor(TRAIN_FRAC * N_s));
N_train_l = max(1, floor(TRAIN_FRAC * N_l));

ze_roll_s_tr  = merge(ze_roll_s{1:N_train_s});
ze_pitch_s_tr = merge(ze_pitch_s{1:N_train_s});
ze_yaw_s_tr   = merge(ze_yaw_s{1:N_train_s});

ze_roll_l_tr  = merge(ze_roll_l{1:N_train_l});
ze_pitch_l_tr = merge(ze_pitch_l{1:N_train_l});
ze_yaw_l_tr   = merge(ze_yaw_l{1:N_train_l});

%% ── Fit ARMAX models ────────────────────────────────────────
fprintf('\n=== Fitting ARMAX[%d %d %d %d] ===\n', ORDERS);

m_roll_s  = armax(ze_roll_s_tr,  ORDERS);
m_pitch_s = armax(ze_pitch_s_tr, ORDERS);
m_yaw_s   = armax(ze_yaw_s_tr,   ORDERS);

m_roll_l  = armax(ze_roll_l_tr,  ORDERS);
m_pitch_l = armax(ze_pitch_l_tr, ORDERS);
m_yaw_l   = armax(ze_yaw_l_tr,   ORDERS);

% Extract fits
fit_sym = [safe_fit(m_roll_s), safe_fit(m_pitch_s), safe_fit(m_yaw_s)];
fit_lop = [safe_fit(m_roll_l), safe_fit(m_pitch_l), safe_fit(m_yaw_l)];

fprintf('\n=== TRAINING FIT COMPARISON ===\n');
fprintf('             Symmetric    Lopsided    Difference\n');
fprintf('  Roll:      %6.2f %%    %6.2f %%    %+6.2f %%\n', ...
    fit_sym(1), fit_lop(1), fit_lop(1)-fit_sym(1));
fprintf('  Pitch:     %6.2f %%    %6.2f %%    %+6.2f %%\n', ...
    fit_sym(2), fit_lop(2), fit_lop(2)-fit_sym(2));
fprintf('  Yaw:       %6.2f %%    %6.2f %%    %+6.2f %%\n', ...
    fit_sym(3), fit_lop(3), fit_lop(3)-fit_sym(3));

%% ── Pole-zero stability ─────────────────────────────────────
fprintf('\n=== STABILITY CHECK ===\n');
poles_s = roots(m_roll_s.A); pole_s = max(abs(poles_s));
poles_l = roots(m_roll_l.A); pole_l = max(abs(poles_l));
fprintf('  Symmetric roll - max |pole| = %.4f (%s)\n', ...
    pole_s, iif(pole_s<1, 'STABLE', 'UNSTABLE'));
fprintf('  Lopsided  roll - max |pole| = %.4f (%s)\n', ...
    pole_l, iif(pole_l<1, 'STABLE', 'UNSTABLE'));

%% ────────────────────────────────────────────────────────────
%% FIGURES
%% ────────────────────────────────────────────────────────────

%% Figure 1: Side-by-side time-domain fit (Roll)
figure('Name', 'Time Domain Comparison', 'Color', 'w', ...
       'Position', [50 50 1400 800]);

subplot(3,2,1); compare(ze_roll_s_tr, m_roll_s);
title(sprintf('Symmetric Roll - %.2f%%', fit_sym(1)));
ylabel('Roll (deg)'); grid on;

subplot(3,2,2); compare(ze_roll_l_tr, m_roll_l);
title(sprintf('Lopsided Roll - %.2f%%', fit_lop(1)));
ylabel('Roll (deg)'); grid on;

subplot(3,2,3); compare(ze_pitch_s_tr, m_pitch_s);
title(sprintf('Symmetric Pitch - %.2f%%', fit_sym(2)));
ylabel('Pitch (deg)'); grid on;

subplot(3,2,4); compare(ze_pitch_l_tr, m_pitch_l);
title(sprintf('Lopsided Pitch - %.2f%%', fit_lop(2)));
ylabel('Pitch (deg)'); grid on;

subplot(3,2,5); compare(ze_yaw_s_tr, m_yaw_s);
title(sprintf('Symmetric Yaw - %.2f%%', fit_sym(3)));
ylabel('Yaw (deg)'); xlabel('Time (s)'); grid on;

subplot(3,2,6); compare(ze_yaw_l_tr, m_yaw_l);
title(sprintf('Lopsided Yaw - %.2f%%', fit_lop(3)));
ylabel('Yaw (deg)'); xlabel('Time (s)'); grid on;

sgtitle('ARMAX Model vs Data: Symmetric (left) vs Lopsided (right)');

%% Figure 2: Fit comparison bar chart
figure('Name', 'Fit Comparison', 'Color', 'w', 'Position', [100 100 1000 500]);
bar([fit_sym; fit_lop]', 'grouped');
set(gca, 'XTickLabel', {'Roll', 'Pitch', 'Yaw'});
ylabel('Fit %'); ylim([0 100]); grid on;
legend('Symmetric payload', 'Lopsided payload', 'Location', 'best');
title('ARMAX Fit % - Payload Symmetry Comparison');

%% Figure 3: Raw data overlay (shows different disturbance profiles)
figure('Name', 'Raw Data Comparison', 'Color', 'w', 'Position', [150 150 1400 800]);

subplot(3,2,1); hold on; grid on;
for i = 1:N_s; plot(raw_s{i}.time_s, raw_s{i}.roll_deg, 'b'); end
title('Symmetric Roll - All Runs'); ylabel('Roll (deg)');
xlabel('Time (s)'); ylim([-30 30]);

subplot(3,2,2); hold on; grid on;
for i = 1:N_l; plot(raw_l{i}.time_s, raw_l{i}.roll_deg, 'r'); end
title('Lopsided Roll - All Runs'); ylabel('Roll (deg)');
xlabel('Time (s)'); ylim([-30 30]);

subplot(3,2,3); hold on; grid on;
for i = 1:N_s; plot(raw_s{i}.time_s, raw_s{i}.pitch_deg, 'b'); end
title('Symmetric Pitch - All Runs'); ylabel('Pitch (deg)');
xlabel('Time (s)'); ylim([-30 30]);

subplot(3,2,4); hold on; grid on;
for i = 1:N_l; plot(raw_l{i}.time_s, raw_l{i}.pitch_deg, 'r'); end
title('Lopsided Pitch - All Runs'); ylabel('Pitch (deg)');
xlabel('Time (s)'); ylim([-30 30]);

subplot(3,2,5); hold on; grid on;
for i = 1:N_s; plot(raw_s{i}.time_s, raw_s{i}.yaw_deg, 'b'); end
title('Symmetric Yaw - All Runs'); ylabel('Yaw (deg)');
xlabel('Time (s)');

subplot(3,2,6); hold on; grid on;
for i = 1:N_l; plot(raw_l{i}.time_s, raw_l{i}.yaw_deg, 'r'); end
title('Lopsided Yaw - All Runs'); ylabel('Yaw (deg)');
xlabel('Time (s)');

sgtitle('Raw IMU Data: Symmetric (left, blue) vs Lopsided (right, red)');

%% Figure 4: Bode comparison (frequency response)
figure('Name', 'Bode Comparison', 'Color', 'w', 'Position', [200 200 1000 600]);
bode(m_roll_s, m_roll_l);
legend('Symmetric payload', 'Lopsided payload', 'Location', 'best');
title('Frequency Response - Roll Dynamics');
grid on;

%% Figure 5: Pole-zero overlay
figure('Name', 'Pole-Zero Comparison', 'Color', 'w', 'Position', [250 250 800 600]);
pzplot(m_roll_s, m_roll_l);
legend('Symmetric', 'Lopsided', 'Location', 'best');
title('Pole-Zero Map - Roll Models');
grid on; axis equal;

%% Figure 6: Residual comparison
figure('Name', 'Residuals', 'Color', 'w', 'Position', [300 300 1400 800]);

subplot(3,2,1); resid(ze_roll_s_tr, m_roll_s);
title('Symmetric Roll'); grid on;
subplot(3,2,2); resid(ze_roll_l_tr, m_roll_l);
title('Lopsided Roll'); grid on;
subplot(3,2,3); resid(ze_pitch_s_tr, m_pitch_s);
title('Symmetric Pitch'); grid on;
subplot(3,2,4); resid(ze_pitch_l_tr, m_pitch_l);
title('Lopsided Pitch'); grid on;
subplot(3,2,5); resid(ze_yaw_s_tr, m_yaw_s);
title('Symmetric Yaw'); grid on;
subplot(3,2,6); resid(ze_yaw_l_tr, m_yaw_l);
title('Lopsided Yaw'); grid on;

sgtitle('Residual Analysis');

%% ── SUMMARY ─────────────────────────────────────────────────
fprintf('\n');
fprintf('================================================================\n');
fprintf('       SYMMETRIC vs LOPSIDED PAYLOAD COMPARISON\n');
fprintf('================================================================\n');
fprintf('  Model orders:  ARMAX[%d %d %d %d]\n', ORDERS);
fprintf('  Sample time:   %.3f s\n', Ts);
fprintf('----------------------------------------------------------------\n');
fprintf('  TRAINING FIT %%:\n');
fprintf('             Symmetric    Lopsided    Difference\n');
fprintf('  Roll:      %6.2f %%    %6.2f %%    %+6.2f %%\n', ...
    fit_sym(1), fit_lop(1), fit_lop(1)-fit_sym(1));
fprintf('  Pitch:     %6.2f %%    %6.2f %%    %+6.2f %%\n', ...
    fit_sym(2), fit_lop(2), fit_lop(2)-fit_sym(2));
fprintf('  Yaw:       %6.2f %%    %6.2f %%    %+6.2f %%\n', ...
    fit_sym(3), fit_lop(3), fit_lop(3)-fit_sym(3));
fprintf('----------------------------------------------------------------\n');
fprintf('  STABILITY:\n');
fprintf('  Symmetric: max pole = %.4f (%s)\n', pole_s, ...
    iif(pole_s<1, 'STABLE', 'UNSTABLE'));
fprintf('  Lopsided:  max pole = %.4f (%s)\n', pole_l, ...
    iif(pole_l<1, 'STABLE', 'UNSTABLE'));
fprintf('================================================================\n');

save('armax_comparison.mat', 'm_roll_s', 'm_pitch_s', 'm_yaw_s', ...
     'm_roll_l', 'm_pitch_l', 'm_yaw_l', 'fit_sym', 'fit_lop', 'ORDERS', 'Ts');
fprintf('\nModels saved to armax_comparison.mat\n');

%% ── HELPERS ─────────────────────────────────────────────────
function [zr, zp, zy, raw, N] = load_runs(data_dir, Ts)
    files = dir(fullfile(data_dir, 'run*.csv'));
    if isempty(files)
        error('No run*.csv files in %s', data_dir);
    end
    N = length(files);
    raw = cell(N, 1);
    zr = cell(N, 1); zp = cell(N, 1); zy = cell(N, 1);
    for i = 1:N
        fpath = fullfile(files(i).folder, files(i).name);
        raw{i} = readtable(fpath, 'CommentStyle', '#');
        d = raw{i};
        u = d.drop_event;
        zr{i} = iddata(detrend(d.roll_deg),  u, Ts);
        zp{i} = iddata(detrend(d.pitch_deg), u, Ts);
        zy{i} = iddata(detrend(d.yaw_deg),   u, Ts);
    end
end

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

function out = iif(cond, a, b)
    if cond; out = a; else; out = b; end
end
