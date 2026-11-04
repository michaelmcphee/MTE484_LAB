%% Lab 2(f) - Beam angle to ball position model
clear; clc;

k2 = 0.0609;

filenames = ["1" "2" "3" "4" "5" "6" "n1" "n2" "n4" "n5" "n6"];
k3_values = zeros(size(filenames));

for idx = 1:length(filenames)

    filename = filenames(idx);
    file = "data/Lab2/partf/" + filename + ".csv";

    data = readmatrix(file);

    t_us      = data(:,1);
    theta_ref = data(:,2);
    y         = data(:,4);

    t = (t_us - t_us(1)) / 1e6;

    gear_angle = theta_ref(1);
    beam_angle = k2 * gear_angle;

    coeffs = polyfit(t, y, 2);
    y_fit = polyval(coeffs, t);

    % Plot data and polynomial fit
    figure;
    plot(t, y, 'b.', 'DisplayName', 'Measured Data');
    hold on;
    plot(t, y_fit, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Quadratic Fit');

    xlabel("Time (s)");
    ylabel("Ball Position");
    title("File " + filename);
    legend("show");
    grid on;
    hold off;

    % Calculate k3
    a = 2 * coeffs(1);
    k3_values(idx) = a / beam_angle;

end

% Display results
table(filenames', k3_values', ...
    'VariableNames', {'Filename', 'k3'})