clear; clc;
%% Plant parameters (from Lab 1 and Lab 2)
k1  = -1.9731;
tau =  0.02387;
k2  =  0.0609;
k3  = -4.838;

%% Sampling period (must match T_S in config.h)
T = 0.001;   % s

%% Inner loop controller C1(s) (from Lab 2 design)
a1 = 1/tau;  b0 = k1/tau;
sig = 14;  wd = 9.5;
pdesi = real(poly([-sig+1j*wd, -sig-1j*wd, -a1]));
f1i = 1;
f0i = pdesi(2) - a1;
g1i = (pdesi(3) - a1*f0i)/b0;
g0i = pdesi(4)/b0;

% Inner D1(z) = (n1i z + n0i) / (m1i z + m0i), forward Euler (s = (z-1)/T)
m1i = f1i;
m0i = f0i*T - f1i;
n1i = g1i;
n0i = g0i*T - g1i;

%% Outer loop controller C2(s) (Lab 3(c), pole placement with integrator)
% C2(s) = (g3o s^3 + g2o s^2 + g1o s + g0o) / (f3o s^3 + f2o s^2 + f1o s + f0o)
%       = (g2o s^2 + g1o s + g0o) / ( s (s^2 + f2o s + f1o) )
% Plant seen by the outer loop (theta = theta_ref assumed): K/s^2
% Augmented with the integrator: K/s^3
K = k2*k3;
sigo = 0.4;  zetao = 0.95;
wno  = sigo/zetao;
wdo  = wno*sqrt(1 - zetao^2);
pdeso = real(poly([-sigo+1j*wdo, -sigo-1j*wdo, -2.5, -5, -10]));
f3o = 1;
f2o = pdeso(2);
f1o = pdeso(3);
f0o = 0;
g3o = 0;
g2o = pdeso(4)/K;
g1o = pdeso(5)/K;
g0o = pdeso(6)/K;

% Outer D2(z), forward Euler (s = (z-1)/T), both polynomials multiplied by T^3
m3o = f3o;
m2o = -3*f3o + f2o*T;
m1o =  3*f3o - 2*f2o*T + f1o*T^2;
m0o = -f3o + f2o*T - f1o*T^2 + f0o*T^3;

n3o = g3o;
n2o = -3*g3o + g2o*T;
n1o =  3*g3o - 2*g2o*T + g1o*T^2;
n0o = -g3o + g2o*T - g1o*T^2 + g0o*T^3;

% Print full-precision values for config.h
format long
disp('Inner D1(z):');  disp([n1i n0i m1i m0i]);
disp('Outer D2(z):');  disp([n3o n2o n1o n0o; m3o m2o m1o m0o]);
format short

%% Run simulation
out = sim('model_Lab3.slx');
y_ref     = out.y_ref;
theta_ref = out.theta_ref;
theta     = out.theta;
y         = out.y;

%% Specs from 3(c)
lo = 0.10;  hi = 0.25;  stepSize = hi - lo;
band    = 0.02 * stepSize;           % 2% settling band = 0.003 m
os_max  = 45;                        % % overshoot limit
ts_max  = 7;                         % s settling limit
os_hi   = hi + os_max/100*stepSize;  % 0.3175 m
os_lo   = lo - os_max/100*stepSize;  % 0.0325 m

%% Figure 1: reference and ball position
figure;
h1 = plot(y_ref, 'k--', 'LineWidth', 1.2); hold on;
h2 = plot(y, 'b', 'LineWidth', 1.2);
h3 = yline(hi + band, 'c--'); yline(hi - band, 'c--');
yline(lo + band, 'c--');      yline(lo - band, 'c--');
h4 = yline(os_hi, 'r:', 'LineWidth', 1);
yline(os_lo, 'r:', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Ball Position (m)');
legend([h1 h2 h3 h4], 'y_{ref}', 'y', '\pm2% band', '45% overshoot limit', 'Location', 'best');
title('Lab 3(c): Ball Position, Discrete Controllers');
grid on;

%% Metrics for each reference edge
t  = y.Time;
yd = y.Data;
r  = interp1(y_ref.Time, y_ref.Data, t, 'previous');
edges = find(abs(diff(r)) > 0.5*stepSize);
edges = edges(t(edges+1) < t(end) - 1);   % drop edges at the end of the sim
nEdge = numel(edges);

os_pct = nan(nEdge,1);  ts = nan(nEdge,1);  peak = nan(nEdge,1);
sse    = nan(nEdge,1);  isUp = false(nEdge,1);

for i = 1:nEdge
    i0 = edges(i) + 1;
    if i < nEdge, i1 = edges(i+1); else, i1 = numel(t); end
    seg_t = t(i0:i1);
    seg_y = yd(i0:i1);
    final = r(i0);
    t0 = t(i0);
    isUp(i) = final > r(edges(i));

    % 2% settling time
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

    fprintf('Edge %d, %s: OS = %.2f %%, peak = %.4f m, ts(2%%) = %.3f s, SS error = %.2e m\n', ...
            i, dir, os_pct(i), peak(i), ts(i), sse(i));
end

%% Summary against 3(c) specs
fprintf('\nWorst-case OS: %.2f %% (spec < %d %%)\n', max(os_pct), os_max);
fprintf('Worst-case ts: %.3f s (spec <= %d s)\n', max(ts, [], 'omitnan'), ts_max);
fprintf('Worst-case |SS error|: %.2e m\n', max(abs(sse)));
fprintf('Edges that never settled: %d\n', sum(isnan(ts)));