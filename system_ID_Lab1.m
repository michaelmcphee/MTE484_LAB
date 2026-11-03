%% Lab 1(e) - System ID from step response data
% Finds every reference transition in the CSV, measures %OS and Tp for
% each step (in both directions), then computes zeta, wn, tau and K1
% per step and averages across all steps.

clear; clc;

% ---------------- User inputs ---------------
filename = "data/serial_data_system_kp15_alpha25.csv";
Kp = -15;              % proportional gain used for this run
peak_search_frac = 0.6;% only search this fraction of each half-period for
                        % the peak, to avoid catching noise near the tail

% ---------------- Load data ----------------
data = readmatrix(filename);

if size(data,2) == 4
    % t_us, theta_ref, theta, V
    t_us      = data(:,1);
    theta_ref = data(:,2);
    theta     = data(:,3);
    t = (t_us - t_us(1)) / 1e6;   % seconds, relative to first sample
    fprintf("Using timestamp column from CSV.\n");
elseif size(data,2) == 3
    % theta_ref, theta, V  (no timestamp - fall back to assumed Ts)
    theta_ref = data(:,1);
    theta     = data(:,2);
    t = (0:size(data,1)-1)' * Ts_assumed;
    fprintf("No timestamp column found. Assuming Ts = %.4f s.\n", Ts_assumed);
else
    error("Unexpected number of columns in %s", filename);
end

%% ---------------- Find step transitions ----------------
trans = find(diff(theta_ref) ~= 0) + 1;   % index of first sample after each step
trans = [1; trans; length(theta_ref)+1];  % bound the segments

nSteps = length(trans) - 2;   % first "segment" (1:trans(2)-1) has no known
                               % pre-step steady value, so it is skipped
results = table('Size', [nSteps 8], ...
    'VariableTypes', {'double','double','double','double','double','double','double','double'}, ...
    'VariableNames', {'step','direction','Tp_s','OS_pct','zeta','wn_rad_s','tau_s','K1'});

fig = figure; hold on; grid on;
plot(t, theta_ref, 'r--', 'LineWidth', 1.1, 'DisplayName', '\theta_{ref}');
plot(t, theta, 'b', 'LineWidth', 1.0, 'DisplayName', '\theta');

row = 0;
for i = 2:length(trans)-1
    segStart = trans(i);
    segEnd   = trans(i+1) - 1;
    seg_t     = t(segStart:segEnd);
    seg_theta = theta(segStart:segEnd);

    initVal = theta(segStart-1);                  % value just before the step
    stepDir = sign(theta_ref(segStart) - initVal); % +1 or -1
    if stepDir == 0
        continue
    end

    % steady-state value: mean of the last 15% of the segment
    tailN = max(1, round(0.15 * length(seg_theta)));
    finalVal = mean(seg_theta(end-tailN+1:end));

    if abs(finalVal - initVal) < 1e-4
        continue   % degenerate step, skip
    end

    % restrict peak search to avoid picking up noise near the tail
    searchN = max(1, round(peak_search_frac * length(seg_theta)));
    searchSeg = seg_theta(1:searchN);

    if stepDir > 0
        [peakVal, peakIdx] = max(searchSeg);
    else
        [peakVal, peakIdx] = min(searchSeg);
    end

    Tp = seg_t(peakIdx) - seg_t(1);
    OS_pct = (peakVal - finalVal) / (finalVal - initVal) * 100;

    if OS_pct <= 0 || Tp <= 0
        continue   % no real overshoot detected for this step, skip
    end

    % ---- zeta, wn, tau, K1 from measured OS and Tp ----
    osFrac = OS_pct / 100;
    zeta = -log(osFrac) / sqrt(pi^2 + log(osFrac)^2);
    wn   = pi / (Tp * sqrt(1 - zeta^2));
    tau  = 1 / (2 * zeta * wn);
    K1   = wn^2 * tau / Kp;

    row = row + 1;
    results(row,:) = {row, stepDir, Tp, OS_pct, zeta, wn, tau, K1};

    plot(t(segStart-1+peakIdx), peakVal, 'ko', 'MarkerFaceColor', 'y', 'HandleVisibility', 'off');
end

results = results(1:row,:);

xlabel("Time (s)");
ylabel("Gear angle (rad)");
title("Kp = -15 Step Response with Marked Peaks");
legend('Location', 'best');

%% ---------------- Summary ----------------
disp(results);

tau_mean = mean(results.tau_s);
K1_mean  = mean(results.K1);

fprintf("Mean tau = %.5f s\n", tau_mean);
fprintf("Mean K1  = %.4f rad/(V*s)\n", K1_mean);
fprintf("Expected ranges: |K1| = 1.50-3.00 rad/(V*s), tau = 0.010-0.035 s\n");