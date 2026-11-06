%% Lab 2(f) - Beam angle to ball position model
clear; clc; close all;
k2 = 0.0609;

use_measured_angle = true;   % true: mean measured gear angle, false: commanded angle

filenames = [
"1a"  "1b"  "1c"  "2a"  "2b"  "2c"  "3a"  "3b"  "3c"
"n1a" "n1b" "n1c" "n2a" "n2b" "n2c" "n3a" "n3b" "n3c"
];
[nr, nc] = size(filenames);
ngroups = nc / 3;            % angles, with 3 trials each
k3_values = nan(nr, nc);
gear_ref  = nan(nr, nc);

figure;
tiledlayout(nr, ngroups, 'TileSpacing', 'compact');

for r = 1:nr
    for g = 1:ngroups
        nexttile((r-1)*ngroups + g);
        hold on; grid on;
        for k = 1:3
            c = 3*(g-1) + k;
            filename = filenames(r, c);
            data = readmatrix("data/Lab2/partf/" + filename + ".csv");
            t_us      = data(:,1);
            theta_ref = data(:,2);
            gear_meas = data(:,3);
            y         = data(:,4);
            t = (t_us - t_us(1)) / 1e6;
            gear_ref(r, c) = theta_ref(1);

            % First and last points define the start and stop
            delta_t = t(end) - t(1);
            delta_y = y(end) - y(1);

            beam_angle = k2 * mean(gear_meas);

            k3_values(r, c) = (2 * delta_y) / (beam_angle * delta_t^2);

            plot(y, '.', 'MarkerSize', 4, 'DisplayName', ...
                sprintf("%s (K3 = %.2f)", filename, k3_values(r, c)));
        end
        title("Gear angle ref = " + gear_ref(r, 3*(g-1)+1));
        xlabel("Time (s)"); ylabel("Ball position y (m)");
        legend('show', 'Location', 'best');
    end
end

%% Averages per angle (3 trials each) and overall
avg_k3 = zeros(nr*ngroups, 1);
std_k3 = zeros(nr*ngroups, 1);
angle  = zeros(nr*ngroups, 1);
for r = 1:nr
    for g = 1:ngroups
        cols = 3*(g-1) + (1:3);
        row = (r-1)*ngroups + g;
        avg_k3(row) = mean(k3_values(r, cols));
        angle(row)  = gear_ref(r, cols(1));
    end
end

results = table(angle, avg_k3, ...
    'VariableNames', {'GearAngleRef', 'AvgK3'})

k3_overall = mean(k3_values(:));
fprintf("Overall average K3 = %.3f \n", k3_overall);