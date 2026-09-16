%% Motor log analysis
% Selects the newest valid device-monitor-YYMMDD-HHMMSS.log capture in
% src/logs, plots the motor signals, and estimates motor parameters from
% each non-zero voltage-step plateau. No auxiliary CSV or helper function
% is required.

%% Editable settings
cfg.LogDirectory = fullfile(fileparts(mfilename('fullpath')), 'src', 'logs');
cfg.VoltageChangeTolerance_V = 0.05;
cfg.SteadyWindow_s = 0.012;
cfg.BaselineWindow_s = 0.012;
cfg.TauFraction = 0.632;
cfg.MinVelocity_rad_s = 1e-6;

% Physical parameters used for the reflected-inertia estimate.
cfg.RobotMass_kg = 927;
cfg.WheelRadius_m = 0.03;
cfg.GearRatio = 9.68;
cfg.MotorCoreInertia_kgm2 = 1.01e-6;

[logPath, data] = selectLatestMonitorLog(cfg.LogDirectory);
[~, logStem, logExtension] = fileparts(logPath);
logName = [logStem logExtension];

time_s = data(:, 1);
voltage = data(:, 4:5);
velocity_rad_s = data(:, 8:9);
current_A = data(:, 15:16);

fprintf('Analyzing newest usable log: %s\n', logPath);

%% Signal plots
figure(1000);
clf;

subplot(2, 1, 1);
plot(time_s, velocity_rad_s(:, 1), time_s, velocity_rad_s(:, 2));
grid on;
legend('Velocity left (rad/s)', 'Velocity right (rad/s)', 'Location', 'best');
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
title(sprintf('Motor velocity — %s', logName), 'Interpreter', 'none');

subplot(2, 1, 2);
plot(time_s, current_A(:, 1), time_s, current_A(:, 2));
grid on;
legend('Current left (A)', 'Current right (A)', 'Location', 'best');
xlabel('Time (s)');
ylabel('Current (A)');
title('Motor current');

%% Step-response estimates
results = table();
wheelNames = ["left", "right"];
reflectedMassInertia_kgm2 = (cfg.RobotMass_kg / 2) * ...
    (cfg.WheelRadius_m / cfg.GearRatio)^2;
totalInertia_kgm2 = cfg.MotorCoreInertia_kgm2 + reflectedMassInertia_kgm2;

for wheel = 1:2
    wheelResults = analyzeWheelSteps(time_s, voltage(:, wheel), ...
        current_A(:, wheel), velocity_rad_s(:, wheel), wheelNames(wheel), ...
        logName, cfg, totalInertia_kgm2);
    results = [results; wheelResults]; %#ok<AGROW>
end

if isempty(results)
    error('analysis:NoVoltageSteps', ...
        'No non-zero voltage-step plateaus were found in %s.', logName);
end

disp('Motor step-response estimates:');
disp(results);

outputPath = fullfile(cfg.LogDirectory, ['motor_dynamics-' logStem '.csv']);
writetable(results, outputPath);
fprintf('Wrote %s\n', outputPath);

if any(isnan(results.Resistance_Ohm) | isnan(results.Tau_s))
    warning('analysis:IncompleteEstimates', ...
        ['One or more resistance or time-constant estimates are unavailable. ' ...
         'Inspect the source log and the displayed step rows.']);
end

function [logPath, data] = selectLatestMonitorLog(logDirectory)
% Select by capture timestamp in the filename, not filesystem modification
% time. This remains correct when logs are copied or checked out together.
if ~isfolder(logDirectory)
    error('analysis:MissingLogDirectory', 'Log directory does not exist: %s', logDirectory);
end

candidates = dir(fullfile(logDirectory, 'device-monitor-*.log'));
candidates = candidates(~[candidates.isdir] & [candidates.bytes] > 0);
if isempty(candidates)
    error('analysis:NoLogs', 'No non-empty device-monitor logs were found in %s.', logDirectory);
end

names = string({candidates.name});
timestamp = regexp(names, '^device-monitor-(\d{6}-\d{6})\.log$', 'tokens', 'once');
isTimestamped = ~cellfun(@isempty, timestamp);
candidates = candidates(isTimestamped);
names = names(isTimestamped);
if isempty(candidates)
    error('analysis:NoTimestampedLogs', ...
        'No non-empty logs use the device-monitor-YYMMDD-HHMMSS.log naming convention.');
end

[~, order] = sort(names, 'descend');
for index = order(:)'
    candidatePath = fullfile(logDirectory, candidates(index).name);
    try
        candidateData = readMonitorLog(candidatePath);
    catch parseError
        warning('analysis:SkippingInvalidLog', 'Skipping %s: %s', ...
            candidates(index).name, parseError.message);
        continue;
    end
    logPath = candidatePath;
    data = candidateData;
    return;
end

error('analysis:NoUsableLogs', 'No timestamped monitor log in %s contains valid data.', logDirectory);
end

function data = readMonitorLog(logPath)
data = readmatrix(logPath, 'FileType', 'text', 'CommentStyle', '%');
data = data(all(isfinite(data), 2), :);
if size(data, 2) < 16
    error('analysis:InvalidLog', 'Expected at least 16 numeric columns, found %d.', size(data, 2));
end
if size(data, 1) < 2
    error('analysis:InvalidLog', 'Expected at least two numeric samples.');
end
data = data(:, 1:16);
if any(diff(data(:, 1)) < 0)
    error('analysis:InvalidLog', 'Time samples are not monotonically increasing.');
end
end

function results = analyzeWheelSteps(time_s, voltage_V, current_A, velocity_rad_s, ...
    wheelName, logName, cfg, totalInertia_kgm2)
changeIndex = find(abs(diff(voltage_V)) > cfg.VoltageChangeTolerance_V) + 1;
segmentStart = [1; changeIndex];
segmentEnd = [changeIndex - 1; numel(time_s)];
results = table();
stepNumber = 0;

for segment = 1:numel(segmentStart)
    startIndex = segmentStart(segment);
    endIndex = segmentEnd(segment);
    plateauVoltage_V = mean(voltage_V(startIndex:endIndex));
    if abs(plateauVoltage_V) <= cfg.VoltageChangeTolerance_V
        continue;
    end
    if startIndex == 1
        warning('analysis:SkippingInitialPlateau', ...
            ['Skipping the initial non-zero plateau for the %s wheel because ' ...
             'it has no preceding baseline from which to measure a step.'], wheelName);
        continue;
    end

    stepNumber = stepNumber + 1;
    baselineStart = find(time_s >= time_s(startIndex) - cfg.BaselineWindow_s, 1, 'first');
    baselineEnd = startIndex - 1;
    baselineVoltage_V = mean(voltage_V(baselineStart:baselineEnd));
    baselineCurrent_A = mean(current_A(baselineStart:baselineEnd));
    voltageStep_V = plateauVoltage_V - baselineVoltage_V;

    responseSign = sign(voltageStep_V);
    response_A = responseSign * (current_A(startIndex:endIndex) - baselineCurrent_A);
    [peakResponse_A, peakOffset] = max(response_A);

    resistance_Ohm = NaN;
    tau_s = NaN;
    inductance_H = NaN;
    if responseSign ~= 0 && peakResponse_A > 0
        resistance_Ohm = abs(voltageStep_V) / peakResponse_A;
        threshold_A = cfg.TauFraction * peakResponse_A;
        crossingOffset = find(response_A(1:peakOffset) >= threshold_A, 1, 'first');
        if ~isempty(crossingOffset)
            crossingIndex = startIndex + crossingOffset - 1;
            crossingTime_s = time_s(crossingIndex);
            if crossingIndex > startIndex
                previousResponse_A = response_A(crossingOffset - 1);
                currentResponse_A = response_A(crossingOffset);
                if currentResponse_A ~= previousResponse_A
                    fraction = (threshold_A - previousResponse_A) / ...
                        (currentResponse_A - previousResponse_A);
                    crossingTime_s = time_s(crossingIndex - 1) + fraction * ...
                        (time_s(crossingIndex) - time_s(crossingIndex - 1));
                end
            end
            tau_s = crossingTime_s - time_s(startIndex);
            inductance_H = resistance_Ohm * tau_s;
        end
    end

    steadyStart = find(time_s >= time_s(endIndex) - cfg.SteadyWindow_s, 1, 'first');
    steadyMask = steadyStart:endIndex;
    steadyVoltage_V = mean(voltage_V(steadyMask));
    steadyCurrent_A = mean(current_A(steadyMask));
    steadyVelocity_rad_s = mean(velocity_rad_s(steadyMask));

    kb_V_rad_s = NaN;
    kt_Nm_A = NaN;
    damping_Nms = NaN;
    if ~isnan(resistance_Ohm) && abs(steadyVelocity_rad_s) > cfg.MinVelocity_rad_s
        kb_V_rad_s = abs((steadyVoltage_V - resistance_Ohm * steadyCurrent_A) / ...
            steadyVelocity_rad_s);
        kt_Nm_A = kb_V_rad_s;
        damping_Nms = abs((steadyCurrent_A * kt_Nm_A) / steadyVelocity_rad_s);
    end

    row = table(string(logName), wheelName, stepNumber, time_s(startIndex), ...
        time_s(endIndex), voltageStep_V, plateauVoltage_V, peakResponse_A, ...
        resistance_Ohm, tau_s, inductance_H, steadyVoltage_V, steadyCurrent_A, ...
        steadyVelocity_rad_s, numel(steadyMask), kb_V_rad_s, kt_Nm_A, ...
        damping_Nms, totalInertia_kgm2, ...
        'VariableNames', {'SourceLog', 'Wheel', 'StepNumber', 'StepStart_s', ...
        'StepEnd_s', 'VoltageStep_V', 'PlateauVoltage_V', 'PeakCurrentChange_A', ...
        'Resistance_Ohm', 'Tau_s', 'Inductance_H', 'SteadyVoltage_V', ...
        'SteadyCurrent_A', 'SteadyVelocity_rad_s', 'SteadySamples', ...
        'Kb_V_rad_s', 'Kt_Nm_A', 'Damping_Nms', 'TotalInertia_kgm2'});
    results = [results; row]; %#ok<AGROW>
end
end