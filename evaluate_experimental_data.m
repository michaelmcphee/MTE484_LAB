%% Plot Lab 3 controller data
data = readmatrix("data/Lab3/partd/serial_data_system.csv");   % change to your file

% Columns: time (us), y_ref, y, theta_ref_unsat, theta_ref_sat, theta, V
t_us          = data(:,1);
y_ref         = data(:,2);
y             = data(:,3);
theta_ref_uns = data(:,4);
theta_ref_sat = data(:,5);
theta         = data(:,6);
V             = data(:,7);
t = (t_us - t_us(1)) / 1e6;   % seconds, starting at 0

%% Specs from 3(c)
lo = 0.10;  hi = 0.25;  stepSize = hi - lo;
band    = 0.02 * stepSize;          % 2% settling band = 0.003 m
os_max  = 45;                       % % overshoot limit
ts_max  = 7;                        % s settling limit
os_hi   = hi + os_max/100*stepSize; % 0.3175 m
os_lo   = lo - os_max/100*stepSize; % 0.0325 m
half_period = 15;   % s, must match HALF_PERIOD_MS / 1000

%% Figure 1: reference and ball position
figure;
h1 = plot(t, y_ref, 'k--', 'LineWidth', 1.2); hold on;
h2 = plot(t, y, 'b', 'LineWidth', 1.2);
h3 = yline(hi + band, 'c--'); yline(hi - band, 'c--');
yline(lo + band, 'c--');      yline(lo - band, 'c--');
h4 = yline(os_hi, 'r:', 'LineWidth', 1);
yline(os_lo, 'r:', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Ball Position (m)');
legend([h1 h2 h3 h4], 'y_{ref}', 'y', '\pm2% band', '45% overshoot limit', 'Location', 'best');
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

os_pct = nan(nEdge,1);  ts = nan(nEdge,1);  peak = nan(nEdge,1);
sse    = nan(nEdge,1);
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

    % Settling: last sample outside the 2% band
    idx = find(abs(seg_y - final) > band, 1, 'last');
    if isempty(idx), ts(i) = 0;
    elseif idx == numel(seg_t), ts(i) = NaN;   % never settled
    else, ts(i) = seg_t(idx+1) - t0;
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

    % Steady-state error: mean error over the last 2 s of the segment
    tail   = seg_t >= seg_t(end) - 2;
    sse(i) = mean(final - seg_y(tail));

    fprintf('Edge %d, %s: OS = %.2f %%, peak = %.4f m, ts = %.3f s, SS error = %.4f m\n', ...
            i, dir, os_pct(i), peak(i), ts(i), sse(i));
end

%% Summary against 3(c) specs
valid = ~isnan(os_pct);
up    = isUp  & valid;
down  = ~isUp & valid;
fprintf('\nStep up   avg OS: %.2f %%   avg ts: %.3f s   max peak: %.4f m (limit %.4f)\n', ...
        mean(os_pct(up)),   mean(ts(up),'omitnan'),   max(peak(up)),   os_hi);
fprintf('Step down avg OS: %.2f %%   avg ts: %.3f s   min peak: %.4f m (limit %.4f)\n', ...
        mean(os_pct(down)), mean(ts(down),'omitnan'), min(peak(down)), os_lo);
fprintf('Worst-case ts: %.3f s (spec %.1f s)\n', max(ts, [], 'omitnan'), ts_max);
fprintf('Worst-case |SS error|: %.4f m (2%% band = %.4f m)\n', max(abs(sse), [], 'omitnan'), band);
fprintf('Edges that never settled: %d\n', sum(valid & isnan(ts)));