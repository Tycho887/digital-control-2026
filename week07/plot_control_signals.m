%% Plot computed PID control signals from the robot log
% Columns 25 and 26 contain the left and right controller outputs in volts.
% The firmware currently records these outputs while applying its separate
% open-loop voltage sequence to the motors.

%% Select capture
% Change logFile to select another capture from src/logs.
logFile = 'device-monitor-261021-170705.log';
repoRoot = fileparts(fileparts(mfilename('fullpath')));
logPath = fullfile(repoRoot, 'src', 'logs', logFile);

assert(isfile(logPath), 'ControlLog:MissingFile', ...
    'Log file does not exist: %s', logPath);
logText = fileread(logPath);
assert(contains(logText, ...
    '% 25-26 computed control signal (left,right) (V)'), ...
    'ControlLog:WrongFormat', ...
    'This capture does not identify columns 25-26 as control signals.');

%% Load numeric samples (ignore serial comments and headers)
data = readmatrix(logPath, 'FileType', 'text', 'CommentStyle', '%');
assert(~isempty(data) && size(data, 2) == 26, ...
    'ControlLog:WrongColumns', 'Expected samples with 26 numeric columns.');
assert(all(isfinite(data), 'all'), 'ControlLog:InvalidSamples', ...
    'The log contains incomplete or non-finite samples.');

time_s = data(:, 1);
controlSignal_V = data(:, 25:26);
assert(all(diff(time_s) > 0), 'ControlLog:MultipleRuns', ...
    'Select a capture containing one run with increasing sample times.');
fprintf('Loaded %d samples from %s\n', size(data, 1), logPath);

%% Plot left and right controller outputs against sample time
figure('Name', 'Robot control signals', 'Color', 'w');
plot(time_s, controlSignal_V(:, 1), 'LineWidth', 1.2);
hold on;
plot(time_s, controlSignal_V(:, 2), 'LineWidth', 1.2);
hold off;
grid on;
xlabel('Time (s)');
ylabel('Computed control signal (V)');
legend('Left motor', 'Right motor', 'Location', 'best');
title({'Computed PID control signals', logFile}, 'Interpreter', 'none');
xlim([time_s(1), time_s(end)]);
