%% Plot Lab 2 controller data
data = readmatrix("data/Lab2/partd.csv");
t_us      = data(:,1);
theta_ref = data(:,2);
theta     = data(:,3);
y         = data(:,4);
V         = data(:,5);
t = (t_us - t_us(1)) / 1e6;   % seconds, starting at 0
band = 0.02 * 1.4;

figure;
subplot(2,1,1);
plot(t, theta_ref, 'r--', 'LineWidth', 1.2); hold on;
plot(t, theta, 'b', 'LineWidth', 1.2);
yline(0.7 + band, 'c--');
yline(0.7 - band, 'c--');
yline(-0.7 + band, 'c--');
yline(-0.7 - band, 'c--');
xlabel("Time (s)");
ylabel("Gear Angle (rad)");
legend("\theta_{ref}", "\theta", 'Location', 'best');
title("Inner-Loop Closed-Loop Step Response");
grid on;

subplot(2,1,2);
plot(t, V, 'k', 'LineWidth', 1.2);
xlabel("Time (s)");
ylabel("Motor Voltage (V)");
title("Control Effort");
grid on;

step_times = [1 2 3 4];
final_vals = [0.7 -0.7 0.7 -0.7];   % up, down, up, down

for k = 1:4
    win = t >= step_times(k) & t < step_times(k) + 1;
    yf  = final_vals(k);

    % Overshoot: peak past the final value, as a percent of the 1.4 rad step
    if yf > 0
        peak = max(theta(win));
    else
        peak = min(theta(win));
    end
    overshoot(k) = (peak - yf) * sign(yf) / 1.4 * 100;

    % Settling time: last sample outside the 2% band
    idx = find(win & abs(theta - yf) > band, 1, 'last');
    settling(k) = t(idx + 1) - step_times(k);
end

up   = [1 3];
down = [2 4];

max_V = max(abs(V));

fprintf('Max |V|: %.3f V\n\n', max_V);
fprintf('Step up   avg overshoot: %.3f %%   avg settling: %.3f s\n', mean(overshoot(up)),   mean(settling(up)));
fprintf('Step down avg overshoot: %.3f %%   avg settling: %.3f s\n', mean(overshoot(down)), mean(settling(down)));