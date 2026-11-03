clear

k1  = -1.9731 ;
tau =  0.02387;

T = 0.001; % descrete sampling time in seconds


a1 = 1/tau;  b0 = k1/tau;               % plant = b0 / (s^2 + a1*s)
sig = 14;  wd = 9.5;                    % chosen complex poles
pdes = real(poly([-sig+1j*wd, -sig-1j*wd, -a1]));   % third pole at -1/tau

f1 = 1;
f0 = pdes(2) - a1;
g1 = (pdes(3) - a1*f0)/b0;g0 = pdes(4)/b0;
C  = tf([g1 g0],[f1 f0]);

m1 = f1;
m0 = f0*T - f1;
n1 = g1;
n0 = g0*T - g1;

simData = sim('model_Lab2.slx');

theta_ref = simData.thetaref;
theta_c    = simData.theta_c;
theta_d = simData.theta_d;

% 2% settling band around the final value (0.7 rad, 1.4 rad step)
final_val = 0.7;
band = 0.02 * 1.4;

figure;
h1 = plot(theta_ref, 'k--', 'LineWidth', 1.2); hold on;
h2 = plot(theta_c, 'b', 'LineWidth', 1.2);
h3 = plot(theta_d, 'r', 'LineWidth', 1.2);
h4 = yline(final_val + band, 'c--', 'LineWidth', 1);
yline(final_val - band, 'c--', 'LineWidth', 1);
xlabel("Time (s)");
ylabel("Gear Angle (rad)");
legend([h1 h2 h3 h4], "\theta_{ref}", "\theta_{c}", "\theta_{d}", "\pm2% band", 'Location', 'best');
title("Simulation with New Controller");
grid on;

t = theta_d.Time;
y = theta_d.Data;

percent_overshoot = (max(y) - 0.7) / 1.4 * 100;

% Settling time: last time outside 0.7 +/- 2% of the 1.4 rad step (step at t = 1 s)
idx = find(t >= 1 & t < 2 & abs(y - 0.7) > band, 1, 'last');
settling_time = t(idx + 1) - 1;
fprintf('Percent Overshoot: %.3f \n', percent_overshoot)
fprintf('Settling Time: %.3f s\n', settling_time);