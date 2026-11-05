%% Motor Potentiometer Reading vs. Motor Angle

% Define calibration data
motor_angle = [0, pi/4];  % no longer having pi/4 and -pi/4 since
motor_pot = [4300, 2033]; % most important to have accurate 0


% Calculate slope and intercept
motor_coefs = polyfit(motor_pot, motor_angle, 1);

% Generate fitted values
motor_x_fit = linspace(min(motor_pot), max(motor_pot), 100);
motor_y_fit = polyval(motor_coefs, motor_x_fit);

% Plot
figure;
plot(motor_pot, motor_angle, 'o', 'DisplayName', 'Data Points');
hold on;
plot(motor_x_fit, motor_y_fit, '-', 'DisplayName', 'Line of Best Fit');

xlabel('Motor Potentiometer Reading');
ylabel('Motor Angle [Radians]');
title('Motor Potentiometer Calibration');
legend show;
grid on;
hold off;


%% Beam Potentiometer Reading vs. Ball Position

% Define calibration data
ball_position = [0, 0.36];
beam_pot = [662.78, 15398.47];

% Calculate slope and intercept
beam_coefs = polyfit(beam_pot, ball_position, 1);

% Generate fitted values
beam_x_fit = linspace(min(beam_pot), max(beam_pot), 100);
beam_y_fit = polyval(beam_coefs, beam_x_fit);

% Plot
figure;
plot(beam_pot, ball_position, 'o', 'DisplayName', 'Data Points');
hold on;
plot(beam_x_fit, beam_y_fit, '-', 'DisplayName', 'Line of Best Fit');

xlabel('Beam Potentiometer Reading');
ylabel('Ball Position [m]');
title('Beam Potentiometer Calibration');
legend show;
grid on;
hold off;


%% Display Calibration Coefficients

fprintf('Motor Calibration:\n');
fprintf('Slope: %.8f\n', motor_coefs(1));
fprintf('Intercept: %.8f\n\n', motor_coefs(2));

fprintf('Beam Calibration:\n');
fprintf('Slope: %.8f\n', beam_coefs(1));
fprintf('Intercept: %.8f\n', beam_coefs(2));