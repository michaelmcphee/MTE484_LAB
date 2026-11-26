%% Plot Lab 3 controller data
data = readmatrix("data/serial_data_system.csv");   % change to your file

% Columns: time (us), y_ref, y, theta_ref_unsat, theta_ref_sat, theta, V
t_us          = data(:,1);
y_ref         = data(:,2);
y             = data(:,3);
theta_ref_uns = data(:,4);
theta_ref_sat = data(:,5);
theta         = data(:,6);
V             = data(:,7);
t = (t_us - t_us(1)) / 1e6;   % seconds, starting at 0

%% Specs
lo = 0.10;  hi = 0.25;  stepSize = hi - lo;

% 3(c): 2% band, 7 s settling, 45% overshoot
band_c  = 0.02 * stepSize;          % 0.003 m
ts_max_c = 7;                       % s
os_max  = 45;                       % %
os_hi   = hi + os_max/100*stepSize; % 0.3175 m
os_lo   = lo - os_max/100*stepSize; % 0.0325 m

% 3(e): wider bands, 8.5 s settling
ss_lo = [0.094 0.106];
ss_hi = [0.240 0.260];
ts_max_e = 8.5;                     % s

half_period = 15;   % s, must match HALF_PERIOD_MS / 1000

%% Figure 1: reference and ball position
figure;
h1 = plot(t, y_ref, 'k--', 'LineWidth', 1.2); hold on;
h2 = plot(t, y, 'b', 'LineWidth', 1.2);
h3 = yline(hi + band_c, 'c--'); yline(hi - band_c, 'c--');
yline(lo + band_c, 'c--');      yline(lo - band_c, 'c--');
h4 = yline(ss_hi(1), 'g--', 'LineWidth', 1); yline(ss_hi(2), 'g--', 'LineWidth', 1);
yline(ss_lo(1), 'g--', 'LineWidth', 1);      yline(ss_lo(2), 'g--', 'LineWidth', 1);
h5 = yline(os_hi, 'r:', 'LineWidth', 1);
yline(os_lo, 'r:', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Ball Position (m)');
legend([h1 h2 h3 h4 h5], 'y_{ref}', 'y', '3(c) \pm2% band', '3(e) band', ...
       '45% overshoot limit', 'Location', 'best');
title('Lab 3(d): Experimental Ball Position');
grid on;

%% Figure 2: gear angle signals and control effort
figure;
subplot(2,1,1);
plot(t, theta_ref_uns, 'm:', 'LineWidth', 1); hold on;
plot(t, theta_ref_sat, 'k--', 'LineWidth', 1.2);
plot(t, theta, 'b', 'LineWidth', 1.2);
yline(0.7, 'r:'); yline(-0.7, 'r:');
xlabel('Time (s)');
ylabel('Gear Angle (rad)');
legend('\theta_{ref} (unsat)', '\theta_{ref,sat}', '\theta', '\pm0.7 rad limit', 'Location', 'best');
title('Gear Angle Signals');
grid on;

subplot(2,1,2);
plot(t, V, 'k', 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('Motor Voltage (V)');
title('Control Effort');
grid on;

%% Metrics for each reference edge
r = y_ref;
edges = find(abs(diff(r)) > 0.5*stepSize);
nEdge = numel(edges);
fprintf('Max |V|: %.3f V\n\n', max(abs(V)));

os_pct = nan(nEdge,1);  peak = nan(nEdge,1);  sse = nan(nEdge,1);
ts_c   = nan(nEdge,1);  ts_e = nan(nEdge,1);
ss_ok  = false(nEdge,1);
isUp   = false(nEdge,1);

for i = 1:nEdge
    i0 = edges(i) + 1;
    if i < nEdge, i1 = edges(i+1); else, i1 = numel(t); end
    seg_t = t(i0:i1);
    seg_y = y(i0:i1);
    final = r(i0);
    t0 = t(i0);
    isUp(i) = final > r(edges(i));

    % Skip a final segment that was cut off before a full half period
    if (seg_t(end) - seg_t(1)) < 0.95*half_period
        fprintf('Edge %d skipped (incomplete segment)\n', i);
        continue;
    end

    % 3(e) band for this level
    if isUp(i), bnd_e = ss_hi; else, bnd_e = ss_lo; end

    % Settling time to the 3(c) 2% band
    idx = find(abs(seg_y - final) > band_c, 1, 'last');
    if isempty(idx), ts_c(i) = 0;
    elseif idx == numel(seg_t), ts_c(i) = NaN;   % never settled
    else, ts_c(i) = seg_t(idx+1) - t0;
    end

    % Settling time to the 3(e) band (enter and not leave)
    idxE = find(seg_y < bnd_e(1) | seg_y > bnd_e(2), 1, 'last');
    if isempty(idxE), ts_e(i) = 0;
    elseif idxE == numel(seg_t), ts_e(i) = NaN;  % never settled
    else, ts_e(i) = seg_t(idxE+1) - t0;
    end

    % Overshoot and peak
    if isUp(i)
        peak(i)   = max(seg_y);
        os_pct(i) = max(peak(i) - final, 0) / stepSize * 100;
        dir = 'rising  (0.10 to 0.25 m)';
    else
        peak(i)   = min(seg_y);
        os_pct(i) = max(final - peak(i), 0) / stepSize * 100;
        dir = 'falling (0.25 to 0.10 m)';
    end

    % Steady-state error and 3(e) band check on the last 2 s of the segment
    tail   = seg_t >= seg_t(end) - 2;
    sse(i) = mean(final - seg_y(tail));
    ss_ok(i) = all(seg_y(tail) >= bnd_e(1) & seg_y(tail) <= bnd_e(2));

    fprintf(['Edge %d, %s: OS = %.2f %%, peak = %.4f m, ts(2%%) = %.3f s, ' ...
             'ts(3e) = %.3f s, SS error = %.4f m, SS in 3(e) band = %d\n'], ...
            i, dir, os_pct(i), peak(i), ts_c(i), ts_e(i), sse(i), ss_ok(i));
end

%% Summary
valid = ~isnan(os_pct);
up    = isUp  & valid;
down  = ~isUp & valid;
fprintf('\nStep up   avg OS: %.2f %%   max peak: %.4f m (limit %.4f)\n', ...
        mean(os_pct(up)),   max(peak(up)),   os_hi);
fprintf('Step down avg OS: %.2f %%   min peak: %.4f m (limit %.4f)\n', ...
        mean(os_pct(down)), min(peak(down)), os_lo);

fprintf('\n--- 3(c): 2%% band (+/-%.4f m), ts <= %.1f s ---\n', band_c, ts_max_c);
fprintf('Worst-case ts: %.3f s\n', max(ts_c, [], 'omitnan'));
fprintf('Worst-case |SS error|: %.4f m\n', max(abs(sse), [], 'omitnan'));
fprintf('Edges that never settled: %d\n', sum(valid & isnan(ts_c)));
fprintf('Edges meeting ts: %d of %d\n', sum(ts_c(valid) <= ts_max_c), sum(valid));

fprintf('\n--- 3(e): 0.094-0.106 m / 0.240-0.260 m, ts <= %.1f s ---\n', ts_max_e);
fprintf('Worst-case ts: %.3f s\n', max(ts_e, [], 'omitnan'));
fprintf('Edges that never settled: %d\n', sum(valid & isnan(ts_e)));
fprintf('Edges meeting ts: %d of %d\n', sum(ts_e(valid) <= ts_max_e), sum(valid));
fprintf('Edges ending inside the band: %d of %d\n', sum(ss_ok(valid)), sum(valid));