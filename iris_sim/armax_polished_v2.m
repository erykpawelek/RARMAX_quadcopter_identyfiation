%% ================================================================
%% ARMAX 3-Way Comparison + Noise Robustness (Polished v2)
%% ================================================================

clear; clc; close all;

%% ── Configuration ────────────────────────────────────────────
DIR_SYM = '/home/philly/armax_data/';
DIR_LOP = '/home/philly/armax_data_lopsided/';
DIR_AGG = '/home/philly/armax_data_aggressive/';
Ts      = 0.01;
ORDERS  = [4 4 4 1];

NOISE_LOW  = 0.005;
NOISE_MED  = 0.020;
NOISE_HIGH = 0.080;

IDENT_THRESHOLD = 60;

C_SYM = [0.20 0.40 0.85];
C_LOP = [0.85 0.35 0.10];
C_AGG = [0.30 0.70 0.30];

%% ── Load data ───────────────────────────────────────────────
fprintf('=== Loading datasets ===\n');

[ze_r_s, ze_p_s, ze_y_s, raw_s, N_s] = load_runs(DIR_SYM, Ts);
fprintf('  Symmetric:  %d runs\n', N_s);

[ze_r_l, ze_p_l, ze_y_l, raw_l, N_l] = load_runs(DIR_LOP, Ts);
fprintf('  Lopsided:   %d runs\n', N_l);

[ze_r_a, ze_p_a, ze_y_a, raw_a, N_a] = load_runs(DIR_AGG, Ts);
fprintf('  Aggressive: %d runs\n', N_a);

%% ── Compute signal amplitudes ───────────────────────────────
fprintf('\n=== Computing signal amplitudes ===\n');
amp_sym = compute_amplitudes(raw_s, N_s);
amp_lop = compute_amplitudes(raw_l, N_l);
amp_agg = compute_amplitudes(raw_a, N_a);

fprintf('  Signal peak-to-peak amplitudes (degrees):\n');
fprintf('              Symmetric    Lopsided     Aggressive\n');
fprintf('    Roll:     %6.3f       %6.3f       %6.3f\n', amp_sym(1), amp_lop(1), amp_agg(1));
fprintf('    Pitch:    %6.3f       %6.3f       %6.3f\n', amp_sym(2), amp_lop(2), amp_agg(2));
fprintf('    Yaw:      %6.3f       %6.3f       %6.3f\n', amp_sym(3), amp_lop(3), amp_agg(3));

%% ── SNR ─────────────────────────────────────────────────────
noise_levels = [NOISE_LOW, NOISE_MED, NOISE_HIGH];
snr_sym = 20 * log10(amp_sym ./ noise_levels);
snr_lop = 20 * log10(amp_lop ./ noise_levels);
snr_agg = 20 * log10(amp_agg ./ noise_levels);

%% ── Merge ──────────────────────────────────────────────────
ze_roll_s_tr  = merge(ze_r_s{:});
ze_pitch_s_tr = merge(ze_p_s{:});
ze_yaw_s_tr   = merge(ze_y_s{:});

ze_roll_l_tr  = merge(ze_r_l{:});
ze_pitch_l_tr = merge(ze_p_l{:});
ze_yaw_l_tr   = merge(ze_y_l{:});

ze_roll_a_tr  = merge(ze_r_a{:});
ze_pitch_a_tr = merge(ze_p_a{:});
ze_yaw_a_tr   = merge(ze_y_a{:});

%% ── Fit clean models ───────────────────────────────────────
fprintf('\n=== Fitting clean ARMAX[%d %d %d %d] ===\n', ORDERS);

m_roll_s  = armax(ze_roll_s_tr,  ORDERS);
m_pitch_s = armax(ze_pitch_s_tr, ORDERS);
m_yaw_s   = armax(ze_yaw_s_tr,   ORDERS);

m_roll_l  = armax(ze_roll_l_tr,  ORDERS);
m_pitch_l = armax(ze_pitch_l_tr, ORDERS);
m_yaw_l   = armax(ze_yaw_l_tr,   ORDERS);

m_roll_a  = armax(ze_roll_a_tr,  ORDERS);
m_pitch_a = armax(ze_pitch_a_tr, ORDERS);
m_yaw_a   = armax(ze_yaw_a_tr,   ORDERS);

fit_sym = [safe_fit(m_roll_s), safe_fit(m_pitch_s), safe_fit(m_yaw_s)];
fit_lop = [safe_fit(m_roll_l), safe_fit(m_pitch_l), safe_fit(m_yaw_l)];
fit_agg = [safe_fit(m_roll_a), safe_fit(m_pitch_a), safe_fit(m_yaw_a)];

%% ── Noise robustness ───────────────────────────────────────
fprintf('\n=== Noise robustness analysis ===\n');

noise_names = {'Low', 'Medium', 'High'};
n_noise = length(noise_levels);

fit_noisy_sym = zeros(3, n_noise);
fit_noisy_lop = zeros(3, n_noise);
fit_noisy_agg = zeros(3, n_noise);

rng(42);

for k = 1:n_noise
    sigma = noise_levels(k);
    fprintf('  Testing %s noise (sigma = %.3f deg)...\n', noise_names{k}, sigma);

    [fit_noisy_sym(1,k), fit_noisy_sym(2,k), fit_noisy_sym(3,k)] = ...
        fit_noisy(ze_r_s, ze_p_s, ze_y_s, sigma, Ts, ORDERS);
    [fit_noisy_lop(1,k), fit_noisy_lop(2,k), fit_noisy_lop(3,k)] = ...
        fit_noisy(ze_r_l, ze_p_l, ze_y_l, sigma, Ts, ORDERS);
    [fit_noisy_agg(1,k), fit_noisy_agg(2,k), fit_noisy_agg(3,k)] = ...
        fit_noisy(ze_r_a, ze_p_a, ze_y_a, sigma, Ts, ORDERS);
end

%% ── Print summary ──────────────────────────────────────────
fprintf('\n');
fprintf('================================================================\n');
fprintf('              COMPLETE RESULTS TABLE\n');
fprintf('================================================================\n');
fprintf('               Symmetric    Lopsided     Aggressive\n');
fprintf('  CLEAN:\n');
fprintf('    Roll:      %6.2f %%     %6.2f %%     %6.2f %%\n', fit_sym(1), fit_lop(1), fit_agg(1));
fprintf('    Pitch:     %6.2f %%     %6.2f %%     %6.2f %%\n', fit_sym(2), fit_lop(2), fit_agg(2));
fprintf('    Yaw:       %6.2f %%     %6.2f %%     %6.2f %%\n', fit_sym(3), fit_lop(3), fit_agg(3));
for k = 1:n_noise
    fprintf('  %s NOISE (sigma=%.3f):\n', upper(noise_names{k}), noise_levels(k));
    fprintf('    Roll:      %6.2f %%     %6.2f %%     %6.2f %%\n', fit_noisy_sym(1,k), fit_noisy_lop(1,k), fit_noisy_agg(1,k));
    fprintf('    Pitch:     %6.2f %%     %6.2f %%     %6.2f %%\n', fit_noisy_sym(2,k), fit_noisy_lop(2,k), fit_noisy_agg(2,k));
    fprintf('    Yaw:       %6.2f %%     %6.2f %%     %6.2f %%\n', fit_noisy_sym(3,k), fit_noisy_lop(3,k), fit_noisy_agg(3,k));
end
fprintf('================================================================\n');

%% ────────────────────────────────────────────────────────────
%% FIGURES
%% ────────────────────────────────────────────────────────────

%% Figure 1: Time-domain fit
figure('Name', '3-Way Time Domain', 'Color', 'w', 'Position', [50 50 1500 900]);
subplot(3,3,1); compare(ze_roll_s_tr, m_roll_s);
title(sprintf('Symmetric Roll - %.1f%%', fit_sym(1))); ylabel('Roll (deg)'); grid on;
subplot(3,3,2); compare(ze_roll_l_tr, m_roll_l);
title(sprintf('Lopsided Roll - %.1f%%', fit_lop(1))); grid on;
subplot(3,3,3); compare(ze_roll_a_tr, m_roll_a);
title(sprintf('Aggressive Roll - %.1f%%', fit_agg(1))); grid on;
subplot(3,3,4); compare(ze_pitch_s_tr, m_pitch_s);
title(sprintf('Symmetric Pitch - %.1f%%', fit_sym(2))); ylabel('Pitch (deg)'); grid on;
subplot(3,3,5); compare(ze_pitch_l_tr, m_pitch_l);
title(sprintf('Lopsided Pitch - %.1f%%', fit_lop(2))); grid on;
subplot(3,3,6); compare(ze_pitch_a_tr, m_pitch_a);
title(sprintf('Aggressive Pitch - %.1f%%', fit_agg(2))); grid on;
subplot(3,3,7); compare(ze_yaw_s_tr, m_yaw_s);
title(sprintf('Symmetric Yaw - %.1f%%', fit_sym(3))); ylabel('Yaw (deg)'); xlabel('Time (s)'); grid on;
subplot(3,3,8); compare(ze_yaw_l_tr, m_yaw_l);
title(sprintf('Lopsided Yaw - %.1f%%', fit_lop(3))); xlabel('Time (s)'); grid on;
subplot(3,3,9); compare(ze_yaw_a_tr, m_yaw_a);
title(sprintf('Aggressive Yaw - %.1f%%', fit_agg(3))); xlabel('Time (s)'); grid on;
sgtitle('ARMAX Model vs Data: 3 Payload Configurations (Clean)', 'FontWeight', 'bold');

%% Figure 2: Clean fit bar chart with threshold
figure('Name', 'Clean Fit Bar', 'Color', 'w', 'Position', [100 100 1200 600]);
bar_data = [fit_sym(:), fit_lop(:), fit_agg(:)];
b = bar(bar_data, 'grouped');
set_bar_colors(b, {C_SYM, C_LOP, C_AGG});
set(gca, 'XTickLabel', {'Roll', 'Pitch', 'Yaw'}, 'FontSize', 12);
ylabel('Fit %', 'FontSize', 12); ylim([0 105]); grid on;
yline(IDENT_THRESHOLD, 'k--', sprintf('%d%% identifiability threshold', IDENT_THRESHOLD), ...
    'LineWidth', 1.5, 'FontSize', 10, 'LabelHorizontalAlignment', 'left');
legend('Symmetric', 'Lopsided', 'Aggressive', 'Location', 'southwest', 'FontSize', 11);
title('Clean ARMAX Fit % - Payload Asymmetry Comparison', 'FontSize', 14);

%% Figure 3: Noise robustness 3x3 with threshold lines
figure('Name', 'Noise Robustness 3-Way', 'Color', 'w', 'Position', [150 150 1500 900]);

axis_names = {'Roll', 'Pitch', 'Yaw'};
data_clean = {fit_sym, fit_lop, fit_agg};
data_noisy = {fit_noisy_sym, fit_noisy_lop, fit_noisy_agg};
config_names = {'Symmetric', 'Lopsided', 'Aggressive'};
config_colors = {C_SYM, C_LOP, C_AGG};

for ax = 1:3
    for cfg = 1:3
        subplot(3, 3, (ax-1)*3 + cfg);

        bd = [data_clean{cfg}(ax), data_noisy{cfg}(ax,:)];
        bb = bar(bd, 'FaceColor', config_colors{cfg});

        set(gca, 'XTickLabel', {'Clean', 'Low', 'Med', 'High'}, 'FontSize', 10);
        ylabel('Fit %');
        ylim([0 105]);
        grid on;
        yline(IDENT_THRESHOLD, 'r--', 'LineWidth', 1.2);
        title(sprintf('%s - %s', config_names{cfg}, axis_names{ax}), 'FontWeight', 'bold');

        for i = 1:length(bd)
            text(i, bd(i)+3, sprintf('%.0f', bd(i)), ...
                 'HorizontalAlignment', 'center', 'FontSize', 9);
        end
    end
end
sgtitle('Noise Robustness - red dashed line = 60% identifiability threshold', 'FontWeight', 'bold');

%% Figure 4: Noise degradation curves
figure('Name', 'Noise Degradation', 'Color', 'w', 'Position', [200 200 1500 500]);

x_idx = 0:n_noise;
x_labels = {'Clean', 'Low', 'Medium', 'High'};

for ax = 1:3
    subplot(1, 3, ax);

    sym_line = [fit_sym(ax), fit_noisy_sym(ax,:)];
    lop_line = [fit_lop(ax), fit_noisy_lop(ax,:)];
    agg_line = [fit_agg(ax), fit_noisy_agg(ax,:)];

    plot(x_idx, sym_line, '-o', 'Color', C_SYM, 'LineWidth', 2.5, ...
         'MarkerSize', 10, 'MarkerFaceColor', C_SYM); hold on;
    plot(x_idx, lop_line, '-s', 'Color', C_LOP, 'LineWidth', 2.5, ...
         'MarkerSize', 10, 'MarkerFaceColor', C_LOP);
    plot(x_idx, agg_line, '-^', 'Color', C_AGG, 'LineWidth', 2.5, ...
         'MarkerSize', 10, 'MarkerFaceColor', C_AGG);

    yline(IDENT_THRESHOLD, 'k--', 'LineWidth', 1.5);

    set(gca, 'XTick', x_idx, 'XTickLabel', x_labels, 'FontSize', 11);
    xlabel('Noise Level');
    ylabel('Fit %');
    title(sprintf('%s Axis', axis_names{ax}), 'FontWeight', 'bold');
    legend('Symmetric', 'Lopsided', 'Aggressive', '60% threshold', 'Location', 'southwest');
    grid on; ylim([0 105]);
end
sgtitle('Noise Degradation: Fit % vs Sensor Noise Level', 'FontWeight', 'bold');

%% Figure 5: SNR analysis
figure('Name', 'SNR Analysis', 'Color', 'w', 'Position', [250 250 1400 500]);

for ax = 1:3
    subplot(1, 3, ax);

    snr_data = [snr_sym(ax,:); snr_lop(ax,:); snr_agg(ax,:)]';
    bb = bar(snr_data, 'grouped');
    set_bar_colors(bb, {C_SYM, C_LOP, C_AGG});

    set(gca, 'XTickLabel', noise_names, 'FontSize', 11);
    xlabel('Noise Level');
    ylabel('SNR (dB)');
    title(sprintf('%s SNR', axis_names{ax}), 'FontWeight', 'bold');
    legend('Symmetric', 'Lopsided', 'Aggressive', 'Location', 'best');
    grid on;
    yline(0, 'k-', 'LineWidth', 0.5);
    yline(20, 'g--', '20 dB', 'LineWidth', 1, 'FontSize', 9);
end
sgtitle('Signal-to-Noise Ratio: Why Aggressive Payload is More Robust', 'FontWeight', 'bold');

%% Figure 6: Signal amplitudes (FIXED)
figure('Name', 'Signal Amplitudes', 'Color', 'w', 'Position', [300 300 1100 500]);
amp_data = [amp_sym(:), amp_lop(:), amp_agg(:)];
b = bar(amp_data, 'grouped');
set_bar_colors(b, {C_SYM, C_LOP, C_AGG});
set(gca, 'XTickLabel', {'Roll', 'Pitch', 'Yaw'}, 'FontSize', 12, 'YScale', 'log');
ylabel('Peak-to-peak amplitude (deg, log scale)');
legend('Symmetric', 'Lopsided', 'Aggressive', 'Location', 'best');
title('Disturbance Signal Amplitudes - Payload Comparison', 'FontWeight', 'bold');
grid on;

%% Figure 7: Asymmetry trend
figure('Name', 'Asymmetry Trend', 'Color', 'w', 'Position', [350 350 1100 500]);
plot([0 1 2], [fit_sym(1), fit_lop(1), fit_agg(1)], '-o', ...
     'Color', C_SYM, 'LineWidth', 2.5, 'MarkerSize', 12, 'MarkerFaceColor', C_SYM); hold on;
plot([0 1 2], [fit_sym(2), fit_lop(2), fit_agg(2)], '-s', ...
     'Color', C_LOP, 'LineWidth', 2.5, 'MarkerSize', 12, 'MarkerFaceColor', C_LOP);
plot([0 1 2], [fit_sym(3), fit_lop(3), fit_agg(3)], '-^', ...
     'Color', C_AGG, 'LineWidth', 2.5, 'MarkerSize', 12, 'MarkerFaceColor', C_AGG);
yline(IDENT_THRESHOLD, 'k--', '60% threshold', 'LineWidth', 1.5, 'FontSize', 10);

xlabel('Payload Asymmetry Level', 'FontSize', 12);
ylabel('ARMAX Fit %', 'FontSize', 12);
title('Identification Quality vs Payload Asymmetry', 'FontWeight', 'bold');
legend('Roll', 'Pitch', 'Yaw', 'Location', 'best', 'FontSize', 11);
grid on;
xticks([0 1 2]);
xticklabels({'Symmetric', 'Lopsided', 'Aggressive'});
ylim([0 105]);
set(gca, 'FontSize', 11);

%% Figure 8: Bode comparison
figure('Name', 'Bode Comparison', 'Color', 'w', 'Position', [400 400 1000 600]);
bode(m_roll_s, m_roll_l, m_roll_a);
legend('Symmetric', 'Lopsided', 'Aggressive', 'Location', 'best');
title('Frequency Response - Roll Dynamics');
grid on;

%% Save
save('armax_polished.mat', ...
     'm_roll_s', 'm_pitch_s', 'm_yaw_s', ...
     'm_roll_l', 'm_pitch_l', 'm_yaw_l', ...
     'm_roll_a', 'm_pitch_a', 'm_yaw_a', ...
     'fit_sym', 'fit_lop', 'fit_agg', ...
     'fit_noisy_sym', 'fit_noisy_lop', 'fit_noisy_agg', ...
     'amp_sym', 'amp_lop', 'amp_agg', ...
     'snr_sym', 'snr_lop', 'snr_agg', ...
     'noise_levels', 'ORDERS', 'Ts');
fprintf('\nResults saved to armax_polished.mat\n');

%% ── HELPERS ─────────────────────────────────────────────────
function set_bar_colors(b, colors)
    % Safely set FaceColor on grouped bar handles
    for i = 1:min(length(b), length(colors))
        if isprop(b(i), 'FaceColor')
            b(i).FaceColor = colors{i};
        end
    end
end

function amp = compute_amplitudes(raw, N)
    amp = zeros(3, 1);
    pk_roll = 0; pk_pitch = 0; pk_yaw = 0;
    for i = 1:N
        d = raw{i};
        post = d.time_s > 5;
        pk_roll  = pk_roll  + (max(d.roll_deg(post))  - min(d.roll_deg(post)));
        pk_pitch = pk_pitch + (max(d.pitch_deg(post)) - min(d.pitch_deg(post)));
        pk_yaw   = pk_yaw   + (max(d.yaw_deg(post))   - min(d.yaw_deg(post)));
    end
    amp(1) = pk_roll  / N;
    amp(2) = pk_pitch / N;
    amp(3) = pk_yaw   / N;
end

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

function [fr, fp_, fy] = fit_noisy(zr_cell, zp_cell, zy_cell, sigma, Ts, ORDERS)
    N = length(zr_cell);
    zr_n = cell(N,1); zp_n = cell(N,1); zy_n = cell(N,1);
    for i = 1:N
        n_r = sigma * randn(size(zr_cell{i}.OutputData));
        n_p = sigma * randn(size(zp_cell{i}.OutputData));
        n_y = sigma * randn(size(zy_cell{i}.OutputData));
        zr_n{i} = iddata(zr_cell{i}.OutputData + n_r, zr_cell{i}.InputData, Ts);
        zp_n{i} = iddata(zp_cell{i}.OutputData + n_p, zp_cell{i}.InputData, Ts);
        zy_n{i} = iddata(zy_cell{i}.OutputData + n_y, zy_cell{i}.InputData, Ts);
    end
    mr = armax(merge(zr_n{:}), ORDERS);
    mp = armax(merge(zp_n{:}), ORDERS);
    my = armax(merge(zy_n{:}), ORDERS);
    fr  = safe_fit(mr);
    fp_ = safe_fit(mp);
    fy  = safe_fit(my);
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
