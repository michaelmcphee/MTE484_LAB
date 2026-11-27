clear; clc; close all;

%% Settings
port       = "COM3";
baudrate   = 115200;
bufferSec  = 45;      % seconds of history shown in the live plot 
refreshSec = 0.05;    % plot refresh interval (s)
sampleRate = 200;     % expected samples per second (5 ms log interval), used to size the buffer

%% Assessment levels (3(e) and demo)
ss_lo  = [0.094 0.106];   % steady-state band at the 0.10 m level
ss_hi  = [0.240 0.260];   % steady-state band at the 0.25 m level
os_hi  = 0.320;           % overshoot limit, rising step
os_lo  = 0.032;           % overshoot limit, falling step
ts_max = 8.5;             % settling time limit (s)
edgeStep = 0.07;          % change in y_ref (m) that counts as a reference edge
yMid   = 0.175;           % y_ref above this value is the high level

%% Live figure
fig = figure('Name', 'Lab 3 live plot (close this window to stop)', 'NumberTitle', 'off');
ax  = axes(fig);  hold(ax, 'on');  grid(ax, 'on');
xl  = [-1e6 1e6];

hBand = patch(ax, [xl(1) xl(2) xl(2) xl(1)], [ss_hi(1) ss_hi(1) ss_hi(2) ss_hi(2)], ...
              [0 0.8 0], 'FaceAlpha', 0.15, 'EdgeColor', 'g', 'LineStyle', '--');
patch(ax, [xl(1) xl(2) xl(2) xl(1)], [ss_lo(1) ss_lo(1) ss_lo(2) ss_lo(2)], ...
      [0 0.8 0], 'FaceAlpha', 0.15, 'EdgeColor', 'g', 'LineStyle', '--', ...
      'HandleVisibility', 'off');
hOS = plot(ax, xl, [os_hi os_hi], 'r:', 'LineWidth', 1.2);
plot(ax, xl, [os_lo os_lo], 'r:', 'LineWidth', 1.2, 'HandleVisibility', 'off');
hTs  = plot(ax, nan, nan, 'm:', 'LineWidth', 1.2);   % legend entry for settling deadlines
hRef = plot(ax, nan, nan, 'k--', 'LineWidth', 1.2);
hY   = plot(ax, nan, nan, 'b',   'LineWidth', 1.2);

ylim(ax, [0 0.35]);  xlim(ax, [0 bufferSec]);
xlabel(ax, 'Time (s)');  ylabel(ax, 'Ball Position (m)');
legend(ax, [hRef hY hBand hOS hTs], 'y_{ref}', 'y', '3(e) steady-state band', ...
       'Overshoot limit', sprintf('Settling deadline (%.1f s after edge)', ts_max), ...
       'Location', 'northwest');

%% Serial setup
s = serialport(port, baudrate);
s.Timeout = 2;
flush(s);
write(s, 1, "uint8");   % start signal

%% Rolling buffer (only the recent past is kept, so memory never grows)
cap  = ceil(2.5 * bufferSec * sampleRate);   % buffer capacity (samples)
keep = floor(0.6 * cap);                     % samples kept when the buffer is trimmed
tt = nan(cap, 1);  rr = nan(cap, 1);  yy = nan(cap, 1);
n = 0;  t0us = [];

tsLines = gobjects(0);   tsTimes = [];   % settling deadline markers
prevRef = [];  tLastEdge = 0;  tLastOut = 0;

%% Live loop (runs until the figure is closed)
plotTimer = tic;
while ishandle(fig)
    try
        line = readline(s);
    catch
        continue;   % read timeout, try again
    end
    values = str2double(split(line, ","));
    if numel(values) ~= 7 || any(isnan(values))
        continue;   % skip partial or corrupted lines
    end

    if isempty(t0us), t0us = values(1); end

    % Trim the oldest samples when the buffer is full
    if n == cap
        tt(1:keep) = tt(cap-keep+1:cap);
        rr(1:keep) = rr(cap-keep+1:cap);
        yy(1:keep) = yy(cap-keep+1:cap);
        n = keep;
    end
    n = n + 1;
    tt(n) = (values(1) - t0us) / 1e6;
    rr(n) = values(2);
    yy(n) = values(3);

    tNow = tt(n);  rNow = rr(n);  yNow = yy(n);

    % Reference edge: start a new settling deadline marker
    if ~isempty(prevRef) && abs(rNow - prevRef) > edgeStep
        tLastEdge = tNow;
        tLastOut  = tNow;
        tsLines(end+1) = plot(ax, [tNow+ts_max tNow+ts_max], [0 0.35], 'm:', ...
                              'LineWidth', 1.2, 'HandleVisibility', 'off');
        tsTimes(end+1) = tNow + ts_max;
    end
    prevRef = rNow;

    % Current band and settling estimate
    if rNow > yMid, b = ss_hi; else, b = ss_lo; end
    inBand = (yNow >= b(1)) && (yNow <= b(2));
    if ~inBand, tLastOut = tNow; end

    % Refresh the plot at a fixed rate
    if toc(plotTimer) > refreshSec
        plotTimer = tic;
        tLo = max(0, tNow - bufferSec);
        iStart = find(tt(1:n) >= tLo, 1);
        if isempty(iStart), iStart = 1; end

        set(hRef, 'XData', tt(iStart:n), 'YData', rr(iStart:n));
        set(hY,   'XData', tt(iStart:n), 'YData', yy(iStart:n));
        xlim(ax, [tLo tLo+bufferSec]);

        % Drop deadline markers that have scrolled out of view
        old = tsTimes < tLo;
        delete(tsLines(old));  tsLines(old) = [];  tsTimes(old) = [];

        if inBand, flag = 'IN BAND'; else, flag = 'OUT OF BAND'; end
        title(ax, sprintf(['t = %.1f s | y_{ref} = %.2f m | y = %.4f m | %s | ' ...
              'settling so far %.2f s (limit %.1f s)'], ...
              tNow, rNow, yNow, flag, tLastOut - tLastEdge, ts_max));
        drawnow limitrate;
    end
end
