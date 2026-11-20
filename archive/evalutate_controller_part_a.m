clear; clc;
%% Plant parameters (from Lab 1 and Lab 2)
k1  = -1.9731;
tau =  0.02387;
k2  =  0.0609;
k3  = -4.838;

%% Inner loop controller C1(s) (from Lab 2 design)
a1 = 1/tau;  b0 = k1/tau;
sig = 14;  wd = 9.5;
pdesi = real(poly([-sig+1j*wd, -sig-1j*wd, -a1]));
f1i = 1;
f0i = pdesi(2) - a1;
g1i = (pdesi(3) - a1*f0i)/b0;
g0i = pdesi(4)/b0;

%% Outer loop controller C2(s) (Lab 3(a))
f1o = 1;
f0o = 2.5;
g1o = -7;
g0o = -2.45;

%% Run simulation
out = sim('model_Lab3_parta.slx');
y_ref     = out.y_ref;
theta_ref = out.theta_ref;
theta     = out.theta;
y         = out.y;

%% Figure 1: reference and ball position
figure;
h1 = plot(y_ref, 'k--', 'LineWidth', 1.2); hold on;
h2 = plot(y, 'b', 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('Ball Position (m)');
legend([h1 h2], 'y_{ref}', 'y', 'Location', 'best');
title('Lab 3(a): Ball Position with Lead Controller');
grid on;
