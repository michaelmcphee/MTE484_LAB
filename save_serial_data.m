clear;

port = "COM3";
baudrate = 115200;
filename = "data/serial_data_system.csv";
duration = 5;  % Recording time in seconds

s = serialport(port, baudrate);

flush(s);
write(s, 1, "uint8");   % start signal

data = [];
tic
while toc < duration
    values = str2double(split(readline(s), ","));
    data(end+1,:) = values';
    fprintf("%d, %d, %d, %d, %d\n", values);
end

writematrix(data, filename);

fprintf("Done. Saved %d samples to:\n%s\n", size(data,1), fullfile(pwd, filename));
