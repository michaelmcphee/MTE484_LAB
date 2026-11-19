clear; clc;

%% Plant parameters (from Lab 1 and Lab 2)
k1  = -1.9731;
tau =  0.02387;
k2  =  0.0609;
k3  = -4.838;

%% Inner loop controller C1(s) (from Lab 2 design)
a1 = 1/tau;  b0 = k1/tau;
sig = 14;  wd = 9.5;
pdes = real(poly([-sig+1j*wd, -sig-1j*wd, -a1]));
f1i = 1;
f0i = pdes(2) - a1;
g1i = (pdes(3) - a1*f0i)/b0;
g0i = pdes(4)/b0;

%% Outer loop controller C1(s) (from Lab 3(a))
f1o = 1;
f0o = 2.5;
g1o = -7;
g0o = -2.45;

%% Run simulation
out = sim('model_Lab3.slx');   % change to your model name

y_ref     = out.y_ref;
theta_ref = out.theta_ref;
theta     = out.theta;
y         = out.y;

%% Assessment levels (from 3(e) and the demo section)
lo = 0.10;  hi = 0.25;  stepSize = hi - lo;
ss_lo = [0.094 0.106];
ss_hi = [0.240 0.260];
os_hi = 0.320;
os_lo = 0.032;

%% Figure 1: reference and ball position
figure;
h1 = plot(y_ref, 'k--', 'LineWidth', 1.2); hold on;
h2 = plot(y, 'b', 'LineWidth', 1.2);
h3 = yline(ss_hi(1), 'c--'); yline(ss_hi(2), 'c--');
yline(ss_lo(1), 'c--');      yline(ss_lo(2), 'c--');
h4 = yline(os_hi, 'r:', 'LineWidth', 1);
yline(os_lo, 'r:', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Ball Position (m)');
legend([h1 h2 h3 h4], 'y_{ref}', 'y', 'Steady-state band', 'Overshoot limit', 'Location', 'best');
title('Lab 3(a): Ball Position with Lead Controller');
grid on;

%% Figure 2: gear angle reference and gear angle
figure;
h1 = plot(theta_ref, 'k--', 'LineWidth', 1.2); hold on;
h2 = plot(theta, 'b', 'LineWidth', 1.2);
h3 = yline(0.7, 'r:', 'LineWidth', 1);
yline(-0.7, 'r:', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Gear Angle (rad)');
legend([h1 h2 h3], '\theta_{ref}', '\theta', '\pm0.7 rad limit', 'Location', 'best');
title('Gear Angle Reference and Response');
grid on;

%% Metrics for each reference edge
t  = y.Time;
yd = y.Data;
r  = interp1(y_ref.Time, y_ref.Data, t, 'previous');
edges = find(abs(diff(r)) > 0.5*stepSize);
edges = edges(t(edges+1) < t(end) - 1);   % drop edges at the end of the sim
band  = 0.02 * stepSize;                  % 3(c) band: 2% of the step

for i = 1:numel(edges)
    i0 = edges(i) + 1;
    if i < numel(edges), i1 = edges(i+1); else, i1 = numel(t); end
    seg_t = t(i0:i1);
    seg_y = yd(i0:i1);
    final = r(i0);
    t0 = t(i0);

    % 3(c) settling: 2% of the step
    idx = find(abs(seg_y - final) > band, 1, 'last');
    if isempty(idx), ts = 0; elseif idx == numel(seg_t), ts = NaN; else, ts = seg_t(idx+1) - t0; end

    % 3(e) settling: 0.094-0.106 m at 0.10 m, 0.240-0.260 m at 0.25 m
    if abs(final - hi) < 1e-6, bandE = 0.010; else, bandE = 0.006; end
    idxE = find(abs(seg_y - final) > bandE, 1, 'last');
    if isempty(idxE), tsE = 0; elseif idxE == numel(seg_t), tsE = NaN; else, tsE = seg_t(idxE+1) - t0; end

    if final > r(edges(i))
        os = (max(seg_y) - final) / stepSize * 100;
        dir = 'rising (0.10 to 0.25 m)';
    else
        os = (final - min(seg_y)) / stepSize * 100;
        dir = 'falling (0.25 to 0.10 m)';
    end
    fprintf('Edge %d, %s: OS = %.2f %%, ts(2%%) = %.3f s, ts(3e band) = %.3f s\n', i, dir, os, ts, tsE);
end