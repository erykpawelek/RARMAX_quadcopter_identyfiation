%% ================================================================
%% ARMAX 3-Way Comparison: Symmetric vs Lopsided vs Aggressive
%% ================================================================
%% Loads three datasets and compares ARMAX identification across:
%%   1. Symmetric payload         (baseline)
%%   2. Mildly lopsided payload   (4cm x 2cm CoM offset)
%%   3. Aggressively lopsided      (8cm x 8cm + cross-inertia)
%%
%% Hypothesis: As payload asymmetry increases, multi-axis coupling
%% increases, and ARMAX identification becomes harder for axes that
%% were originally unexcited (yaw and pitch).
%% ================================================================

clear; clc; close all;

%% ── Configuration ────────────────────────────────────────────
DIR_SYM = '/home/philly/armax_data/';
DIR_LOP = '/home/philly/armax_data_lopsided/';
DIR_AGG = '/home/philly/armax_data_aggressive/';
Ts      = 0.01;
ORDERS  = [4 4 4 1];

%% ── Load all three datasets ─────────────────────────────────
fprintf('=== Loading datasets ===\n');

[ze_r_s, ze_p_s, ze_y_s, raw_s, N_s] = load_runs(DIR_SYM, Ts);
fprintf('  Symmetric:  %d runs\n', N_s);

[ze_r_l, ze_p_l, ze_y_l, raw_l, N_l] = load_runs(DIR_LOP, Ts);
fprintf('  Lopsided:   %d runs\n', N_l);

[ze_r_a, ze_p_a, ze_y_a, raw_a, N_a] = load_runs(DIR_AGG, Ts);
fprintf('  Aggressive: %d runs\n', N_a);

%% ── Merge for training ──────────────────────────────────────
ze_roll_s_tr  = merge(ze_r_s{:});
ze_pitch_s_tr = merge(ze_p_s{:});
ze_yaw_s_tr   = merge(ze_y_s{:});

ze_roll_l_tr  = merge(ze_r_l{:});
ze_pitch_l_tr = merge(ze_p_l{:});
ze_yaw_l_tr   = merge(ze_y_l{:});

ze_roll_a_tr  = merge(ze_r_a{:});
ze_pitch_a_tr = merge(ze_p_a{:});
ze_yaw_a_tr   = merge(ze_y_a{:});

%% ── Fit ARMAX on all 9 axis/payload combinations ────────────
fprintf('\n=== Fitting ARMAX[%d %d %d %d] ===\n', ORDERS);

m_roll_s  = armax(ze_roll_s_tr,  ORDERS);
m_pitch_s = armax(ze_pitch_s_tr, ORDERS);
m_yaw_s   = armax(ze_yaw_s_tr,   ORDERS);

m_roll_l  = armax(ze_roll_l_tr,  ORDERS);
m_pitch_l = armax(ze_pitch_l_tr, ORDERS);
m_yaw_l   = armax(ze_yaw_l_tr,   ORDERS);

m_roll_a  = armax(ze_roll_a_tr,  ORDERS);
m_pitch_a = armax(ze_pitch_a_tr, ORDERS);
m_yaw_a   = armax(ze_yaw_a_tr,   ORDERS);

% Extract fits
fit_sym = [safe_fit(m_roll_s), safe_fit(m_pitch_s), safe_fit(m_yaw_s)];
fit_lop = [safe_fit(m_roll_l), safe_fit(m_pitch_l), safe_fit(m_yaw_l)];
fit_agg = [safe_fit(m_roll_a), safe_fit(m_pitch_a), safe_fit(m_yaw_a)];

%% ── Print summary ───────────────────────────────────────────
fprintf('\n');
fprintf('================================================================\n');
fprintf('             ARMAX FIT %% - 3-WAY COMPARISON\n');
fprintf('================================================================\n');
fprintf('               Symmetric    Lopsided     Aggressive\n');
fprintf('  Roll:        %6.2f %%    %6.2f %%    %6.2f %%\n', ...
    fit_sym(1), fit_lop(1), fit_agg(1));
fprintf('  Pitch:       %6.2f %%    %6.2f %%    %6.2f %%\n', ...
    fit_sym(2), fit_lop(2), fit_agg(2));
fprintf('  Yaw:         %6.2f %%    %6.2f %%    %6.2f %%\n', ...
    fit_sym(3), fit_lop(3), fit_agg(3));
fprintf('================================================================\n');

%% ────────────────────────────────────────────────────────────
%% FIGURES
%% ────────────────────────────────────────────────────────────

%% Figure 1: Time-domain fit comparison (3 columns x 3 rows)
figure('Name', '3-Way Time Domain', 'Color', 'w', ...
       'Position', [50 50 1500 900]);

subplot(3,3,1); compare(ze_roll_s_tr, m_roll_s);
title(sprintf('Sym Roll - %.1f%%', fit_sym(1)));
ylabel('Roll (deg)'); grid on;

subplot(3,3,2); compare(ze_roll_l_tr, m_roll_l);
title(sprintf('Lopsided Roll - %.1f%%', fit_lop(1)));
grid on;

subplot(3,3,3); compare(ze_roll_a_tr, m_roll_a);
title(sprintf('Aggressive Roll - %.1f%%', fit_agg(1)));
grid on;

subplot(3,3,4); compare(ze_pitch_s_tr, m_pitch_s);
title(sprintf('Sym Pitch - %.1f%%', fit_sym(2)));
ylabel('Pitch (deg)'); grid on;

subplot(3,3,5); compare(ze_pitch_l_tr, m_pitch_l);
title(sprintf('Lopsided Pitch - %.1f%%', fit_lop(2)));
grid on;

subplot(3,3,6); compare(ze_pitch_a_tr, m_pitch_a);
title(sprintf('Aggressive Pitch - %.1f%%', fit_agg(2)));
grid on;

subplot(3,3,7); compare(ze_yaw_s_tr, m_yaw_s);
title(sprintf('Sym Yaw - %.1f%%', fit_sym(3)));
ylabel('Yaw (deg)'); xlabel('Time (s)'); grid on;

subplot(3,3,8); compare(ze_yaw_l_tr, m_yaw_l);
title(sprintf('Lopsided Yaw - %.1f%%', fit_lop(3)));
xlabel('Time (s)'); grid on;

subplot(3,3,9); compare(ze_yaw_a_tr, m_yaw_a);
title(sprintf('Aggressive Yaw - %.1f%%', fit_agg(3)));
xlabel('Time (s)'); grid on;

sgtitle('ARMAX Model vs Data: 3 Payload Configurations');

%% Figure 2: Bar chart fit comparison
figure('Name', 'Fit Comparison', 'Color', 'w', ...
       'Position', [100 100 1200 500]);
bar([fit_sym; fit_lop; fit_agg]', 'grouped');
set(gca, 'XTickLabel', {'Roll', 'Pitch', 'Yaw'});
ylabel('Fit %'); ylim([0 100]); grid on;
legend('Symmetric', 'Lopsided', 'Aggressive', 'Location', 'best');
title('ARMAX Fit % - Payload Asymmetry Comparison');

%% Figure 3: Raw data comparison
figure('Name', 'Raw Data 3-Way', 'Color', 'w', ...
       'Position', [150 150 1500 900]);

cmap = lines(max([N_s, N_l, N_a]));

subplot(3,3,1); hold on; grid on;
for i = 1:N_s; plot(raw_s{i}.time_s, raw_s{i}.roll_deg, 'Color', cmap(i,:)); end
title('Symmetric Roll'); ylabel('Roll (deg)');

subplot(3,3,2); hold on; grid on;
for i = 1:N_l; plot(raw_l{i}.time_s, raw_l{i}.roll_deg, 'Color', cmap(i,:)); end
title('Lopsided Roll');

subplot(3,3,3); hold on; grid on;
for i = 1:N_a; plot(raw_a{i}.time_s, raw_a{i}.roll_deg, 'Color', cmap(i,:)); end
title('Aggressive Roll');

subplot(3,3,4); hold on; grid on;
for i = 1:N_s; plot(raw_s{i}.time_s, raw_s{i}.pitch_deg, 'Color', cmap(i,:)); end
title('Symmetric Pitch'); ylabel('Pitch (deg)');

subplot(3,3,5); hold on; grid on;
for i = 1:N_l; plot(raw_l{i}.time_s, raw_l{i}.pitch_deg, 'Color', cmap(i,:)); end
title('Lopsided Pitch');

subplot(3,3,6); hold on; grid on;
for i = 1:N_a; plot(raw_a{i}.time_s, raw_a{i}.pitch_deg, 'Color', cmap(i,:)); end
title('Aggressive Pitch');

subplot(3,3,7); hold on; grid on;
for i = 1:N_s; plot(raw_s{i}.time_s, raw_s{i}.yaw_deg, 'Color', cmap(i,:)); end
title('Symmetric Yaw'); ylabel('Yaw (deg)'); xlabel('Time (s)');

subplot(3,3,8); hold on; grid on;
for i = 1:N_l; plot(raw_l{i}.time_s, raw_l{i}.yaw_deg, 'Color', cmap(i,:)); end
title('Lopsided Yaw'); xlabel('Time (s)');

subplot(3,3,9); hold on; grid on;
for i = 1:N_a; plot(raw_a{i}.time_s, raw_a{i}.yaw_deg, 'Color', cmap(i,:)); end
title('Aggressive Yaw'); xlabel('Time (s)');

sgtitle('Raw IMU Data: Increasing Payload Asymmetry');

%% Figure 4: Bode comparison (Roll axis)
figure('Name', 'Bode Comparison', 'Color', 'w', 'Position', [200 200 1000 600]);
bode(m_roll_s, m_roll_l, m_roll_a);
legend('Symmetric', 'Lopsided', 'Aggressive', 'Location', 'best');
title('Frequency Response - Roll Dynamics');
grid on;

%% Figure 5: Pole-zero overlay
figure('Name', 'Pole-Zero Comparison', 'Color', 'w', ...
       'Position', [250 250 1400 500]);
subplot(1,3,1); pzmap(m_roll_s); title('Symmetric Roll'); axis equal; grid on;
subplot(1,3,2); pzmap(m_roll_l); title('Lopsided Roll'); axis equal; grid on;
subplot(1,3,3); pzmap(m_roll_a); title('Aggressive Roll'); axis equal; grid on;
sgtitle('Pole-Zero Maps - Roll Models');

%% Figure 6: Trend analysis
figure('Name', 'Trend Analysis', 'Color', 'w', 'Position', [300 300 1100 500]);
asymmetry = [0, 1, 2];   % 0=symmetric, 1=mild, 2=aggressive
plot(asymmetry, fit_sym(1), 'bo-', 'LineWidth', 2, 'MarkerSize', 10); hold on;
plot(asymmetry, fit_lop(1), 'bo-', 'LineWidth', 2, 'MarkerSize', 10);
plot([0 1 2], [fit_sym(1), fit_lop(1), fit_agg(1)], 'b-', 'LineWidth', 2, 'DisplayName', 'Roll');
plot([0 1 2], [fit_sym(2), fit_lop(2), fit_agg(2)], 'r-', 'LineWidth', 2, 'DisplayName', 'Pitch');
plot([0 1 2], [fit_sym(3), fit_lop(3), fit_agg(3)], 'g-', 'LineWidth', 2, 'DisplayName', 'Yaw');
xlabel('Payload Asymmetry Level (0=symmetric, 1=mild, 2=aggressive)');
ylabel('ARMAX Fit %');
title('Identification Difficulty vs Payload Asymmetry');
legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');
grid on; xticks([0 1 2]);
xticklabels({'Symmetric', 'Lopsided', 'Aggressive'});
ylim([0 100]);

%% Save models
save('armax_3way_comparison.mat', ...
     'm_roll_s', 'm_pitch_s', 'm_yaw_s', ...
     'm_roll_l', 'm_pitch_l', 'm_yaw_l', ...
     'm_roll_a', 'm_pitch_a', 'm_yaw_a', ...
     'fit_sym', 'fit_lop', 'fit_agg', 'ORDERS', 'Ts');
fprintf('\nModels saved to armax_3way_comparison.mat\n');

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
