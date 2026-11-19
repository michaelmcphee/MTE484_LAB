clear;
port = "COM3";
baudrate = 115200;
filename = "data/serial_data_system.csv";
duration = 90;  % Recording time in seconds (3 periods of the 30 s square wave)

s = serialport(port, baudrate);
flush(s);
write(s, 1, "uint8");   % start signal

% Columns: time (us), y_ref, y, theta_ref_unsat, theta_ref_sat, theta, V
data = [];
tic
while toc < duration
    values = str2double(split(readline(s), ","));
    if numel(values) ~= 7 || any(isnan(values))
        continue;   % skip partial or corrupted lines
    end
    data(end+1,:) = values';
    fprintf("%d, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", values);
end

% Stop the motor path cleanly by closing the port
clear s;

writematrix(data, filename);
fprintf("Done. Saved %d samples to:\n%s\n", size(data,1), fullfile(pwd, filename));