%% Report and plot control signals, measured velocity, and set point error
% Use captures from the PI forward-speed firmware (column 3 is wheel m/s).
% Earlier open-loop voltage captures use different set point units.

%% Capture and geometry used when recording it
logFile = 'device-monitor-261023-220230.log';
gearRatio = 9.6;
wheelRadius_m = 0.0315; % Full precision; the log header rounds this to 0.03.
repoRoot = fileparts(fileparts(mfilename('fullpath')));
logPath = fullfile(repoRoot, 'src', 'logs', logFile);
assert(isfile(logPath), 'TrackingLog:MissingFile', ...
    'Log file does not exist: %s', logPath);
assert(gearRatio > 0 && wheelRadius_m > 0, ...
    'TrackingLog:InvalidGeometry', 'Gear ratio and wheel radius must be positive.');

%% Load samples, ignoring comments and serial replies such as "version ..."
logText = fileread(logPath);
assert(contains(logText, ...
    '% 25-26 computed control signal (left,right) (V)'), ...
    'TrackingLog:WrongFormat', 'Expected a log with control signals in columns 25-26.');
logLines = splitlines(string(logText));
data = zeros(numel(logLines), 26);
sampleCount = 0;
for lineIndex = 1:numel(logLines)
    line = strtrim(char(logLines(lineIndex)));
    if isempty(regexp(line, '^[+-]?(?:\d|\.\d)', 'once'))
        continue;
    end
    row = sscanf(line, '%f').';
    assert(numel(row) == 26 && all(isfinite(row)) && ...
        ~isempty(regexp(line, '^[\s0-9eE+.\-]+$', 'once')), ...
        'TrackingLog:MalformedSample', 'Invalid numeric sample on line %d.', lineIndex);
    sampleCount = sampleCount + 1;
    data(sampleCount, :) = row;
end
data = data(1:sampleCount, :);
assert(sampleCount > 1, 'TrackingLog:NoSamples', 'The capture has no usable run.');
time_s = data(:, 1);
assert(all(diff(time_s) > 0), 'TrackingLog:MultipleRuns', ...
    'Select a capture containing one run with increasing sample times.');
sentCount = regexp(logText, '% sent (\d+) log lines', 'tokens');
assert(numel(sentCount) == 1 && str2double(sentCount{1}{1}) == sampleCount, ...
    'TrackingLog:IncompleteTransfer', 'Sample count does not match a completed log transfer.');
capacity = regexp(logText, '% maximum log samples (\d+)', 'tokens', 'once');
if contains(logText, '% WARNING: log buffer filled') || ...
        (~isempty(capacity) && sampleCount >= str2double(capacity{1}))
    warning('TrackingLog:CapacityReached', ...
        'This capture reached the log buffer capacity; final run samples may be missing.');
end

%% Match the controller's forward-positive motor speed and error convention
setPointWheel_m_s = data(:, 3);
setPointMotor_rad_s = setPointWheel_m_s * gearRatio / wheelRadius_m;
measuredVelocity_rad_s = data(:, 8:9); % Already sign-corrected in the log.
controlSignal_V = data(:, 25:26);
setPointError_rad_s = setPointMotor_rad_s - measuredVelocity_rad_s;

% Per-sample report is retained as a table in the MATLAB workspace.
samples = table(time_s, setPointWheel_m_s, setPointMotor_rad_s, ...
    controlSignal_V(:, 1), controlSignal_V(:, 2), ...
    measuredVelocity_rad_s(:, 1), measuredVelocity_rad_s(:, 2), ...
    setPointError_rad_s(:, 1), setPointError_rad_s(:, 2), ...
    'VariableNames', {'Time_s', 'SetPointWheel_m_s', 'SetPointMotor_rad_s', ...
    'ControlLeft_V', 'ControlRight_V', 'MeasuredLeft_rad_s', 'MeasuredRight_rad_s', ...
    'ErrorLeft_rad_s', 'ErrorRight_rad_s'});

summary = table(["Left"; "Right"], mean(controlSignal_V, 1).', ...
    max(abs(controlSignal_V), [], 1).', mean(measuredVelocity_rad_s, 1).', ...
    mean(setPointError_rad_s, 1).', sqrt(mean(setPointError_rad_s.^2, 1)).', ...
    max(abs(setPointError_rad_s), [], 1).', ...
    'VariableNames', {'Motor', 'MeanControl_V', 'PeakAbsControl_V', ...
    'MeanMeasuredVelocity_rad_s', 'MeanError_rad_s', 'RMSError_rad_s', ...
    'PeakAbsError_rad_s'});
fprintf('Loaded %d samples, %.4f-%.4f s, from %s\n', ...
    sampleCount, time_s(1), time_s(end), logPath);
fprintf('Error = motor speed set point - measured motor speed (rad/s).\n');
fprintf('Summary over the entire recorded capture, including rest intervals:\n');
disp(summary);

%% Compare both motors over time
figure('Name', 'Robot control and tracking', 'Color', 'w', ...
    'Position', [100, 100, 1100, 800]);
layout = tiledlayout(3, 1, 'TileSpacing', 'compact');
axesHandles = gobjects(3, 1);
axesHandles(1) = nexttile(layout);
plot(time_s, controlSignal_V, 'LineWidth', 1.1);
grid on;
ylabel('Control signal (V)');
legend('Left', 'Right', 'Location', 'best');

axesHandles(2) = nexttile(layout);
plot(time_s, measuredVelocity_rad_s, 'LineWidth', 1.1);
hold on;
plot(time_s, setPointMotor_rad_s, 'k--', 'LineWidth', 1.3);
hold off;
grid on;
ylabel('Motor speed (rad/s)');
legend('Measured left', 'Measured right', 'Set point', 'Location', 'best');

axesHandles(3) = nexttile(layout);
plot(time_s, setPointError_rad_s, 'LineWidth', 1.1);
yline(0, 'k:');
grid on;
ylabel('Set point error (rad/s)');
xlabel('Time (s)');
legend('Left', 'Right', 'Location', 'best');
linkaxes(axesHandles, 'x');
xlim(axesHandles(1), [time_s(1), time_s(end)]);
title(layout, {'Control and velocity tracking', logFile}, 'Interpreter', 'none');
