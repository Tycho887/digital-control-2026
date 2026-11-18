% Plot the latest compatible robot log and save a PNG beside the capture.

scriptDirectory = fileparts(mfilename('fullpath'));
sourceDirectory = fileparts(scriptDirectory);
logDirectory = fullfile(sourceDirectory, 'logs');

% These columns cover every signal shown in the controller tracking plots.
signalColumns = [1, 3, 30, 31, 32, 33, 34, 42, 43];
[data, logPath, turnRateCalibration] = loadNewestFormat4Log(logDirectory, signalColumns);

if turnRateCalibration
    addpath(scriptDirectory);
    [figureHandle, pngPath, calibration] = plot_turn_rate_calibration(logPath);
    time_s = data(:,1);
else

time_s = data(:, 1);
referenceSpeed_m_s = data(:, 3);
wheelSetpoint_rad_s = data(:, 30:31);
referenceYawRate_rad_s = data(:, 32);
estimatedSpeed_m_s = data(:, 33);
estimatedYawRate_rad_s = data(:, 34);
filteredWheelFeedback_rad_s = data(:, 42:43);

[~, logName, ~] = fileparts(logPath);
figureHandle = figure( ...
    'Color', 'w', ...
    'Name', ['Robot log tracking: ', logName], ...
    'NumberTitle', 'off');
layout = tiledlayout(figureHandle, 3, 1, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
sgtitle(layout, ['Robot tracking: ', logName], ...
    'Interpreter', 'none', 'Color', 'k');

speedAxes = nexttile(layout, 1);
set(speedAxes, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
    'GridColor', [0.25, 0.25, 0.25], 'GridAlpha', 0.18);
plot(speedAxes, time_s, referenceSpeed_m_s, 'k--', ...
    'LineWidth', 1.2, 'DisplayName', 'Programmed reference');
hold(speedAxes, 'on');
plot(speedAxes, time_s, estimatedSpeed_m_s, ...
    'Color', [0.0000, 0.4470, 0.7410], 'LineWidth', 1.1, ...
    'DisplayName', 'Kalman estimate');
hold(speedAxes, 'off');
ylabel(speedAxes, 'Forward speed (m/s)', 'Color', 'k');
title(speedAxes, 'Chassis forward-speed tracking', 'Color', 'k');
grid(speedAxes, 'on');
legend(speedAxes, 'Location', 'best', 'Color', 'w', ...
    'TextColor', 'k', 'EdgeColor', [0.3, 0.3, 0.3]);

yawAxes = nexttile(layout, 2);
set(yawAxes, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
    'GridColor', [0.25, 0.25, 0.25], 'GridAlpha', 0.18);
plot(yawAxes, time_s, referenceYawRate_rad_s, 'k--', ...
    'LineWidth', 1.2, 'DisplayName', 'Programmed reference');
hold(yawAxes, 'on');
plot(yawAxes, time_s, estimatedYawRate_rad_s, ...
    'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 1.1, ...
    'DisplayName', 'Kalman estimate');
hold(yawAxes, 'off');
ylabel(yawAxes, 'Yaw rate (rad/s)', 'Color', 'k');
title(yawAxes, 'Chassis yaw-rate tracking', 'Color', 'k');
grid(yawAxes, 'on');
legend(yawAxes, 'Location', 'best', 'Color', 'w', ...
    'TextColor', 'k', 'EdgeColor', [0.3, 0.3, 0.3]);

wheelAxes = nexttile(layout, 3);
set(wheelAxes, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
    'GridColor', [0.25, 0.25, 0.25], 'GridAlpha', 0.18);
wheelColors = lines(2);
hold(wheelAxes, 'on');
plot(wheelAxes, time_s, wheelSetpoint_rad_s(:, 1), ...
    'Color', wheelColors(1, :), 'LineWidth', 1.2, ...
    'DisplayName', 'Left setpoint');
plot(wheelAxes, time_s, filteredWheelFeedback_rad_s(:, 1), '--', ...
    'Color', wheelColors(1, :), 'LineWidth', 1.0, ...
    'DisplayName', 'Left filtered feedback');
plot(wheelAxes, time_s, wheelSetpoint_rad_s(:, 2), ...
    'Color', wheelColors(2, :), 'LineWidth', 1.2, ...
    'DisplayName', 'Right setpoint');
plot(wheelAxes, time_s, filteredWheelFeedback_rad_s(:, 2), '--', ...
    'Color', wheelColors(2, :), 'LineWidth', 1.0, ...
    'DisplayName', 'Right filtered feedback');
hold(wheelAxes, 'off');
xlabel(wheelAxes, 'Time (s)', 'Color', 'k');
ylabel(wheelAxes, 'Motor speed (rad/s)', 'Color', 'k');
title(wheelAxes, 'Wheel-speed setpoint tracking', 'Color', 'k');
grid(wheelAxes, 'on');
legend(wheelAxes, 'Location', 'best', 'Color', 'w', ...
    'TextColor', 'k', 'EdgeColor', [0.3, 0.3, 0.3]);

linkaxes([speedAxes, yawAxes, wheelAxes], 'x');
% Apply the light theme after plotting/linking so MATLAB's active theme does
% not restore dark axes or light tick labels in the exported image.
set(figureHandle, 'Color', 'w');
set([speedAxes, yawAxes, wheelAxes], 'Color', 'w', ...
    'XColor', 'k', 'YColor', 'k', ...
    'GridColor', [0.25, 0.25, 0.25], 'GridAlpha', 0.18);

pngPath = fullfile(logDirectory, [logName, '_tracking.png']);
exportgraphics(figureHandle, pngPath, 'Resolution', 160);
fprintf('Selected log: %s\n', logPath);
fprintf('Plotted %d samples over %.3f s.\n', ...
    numel(time_s), time_s(end) - time_s(1));
fprintf('Saved figure: %s\n', pngPath);
end

function [data, selectedPath, turnRateCalibration] = loadNewestFormat4Log(logDirectory, signalColumns)
    logs = dir(fullfile(logDirectory, '*.log'));
    if isempty(logs)
        error('plot_latest_log:NoLogs', ...
            'No .log files found in %s.', logDirectory);
    end

    [~, newestFirst] = sort([logs.datenum], 'descend');
    requiredColumnCount = max(signalColumns);

    for index = newestFirst
        candidatePath = fullfile(logDirectory, logs(index).name);
        try
            logText = fileread(candidatePath);
            if contains(logText, '% CAL turn-rate-calibration')
                addpath(fileparts(mfilename('fullpath')));
                capture = read_turn_rate_calibration(candidatePath);
                data = capture.data;
                selectedPath = candidatePath;
                turnRateCalibration = true;
                return;
            end
            if ~contains(lower(logText), '% log format 4')
                continue;
            end

            candidateData = readmatrix(candidatePath, ...
                'FileType', 'text', 'CommentStyle', '%');
        catch
            continue;
        end

        if isempty(candidateData) || size(candidateData, 2) < requiredColumnCount
            continue;
        end

        % Ignore comments and malformed samples based only on plotted signals;
        % unused diagnostics may legitimately contain NaN values.
        validSamples = all(isfinite(candidateData(:, signalColumns)), 2);
        candidateData = candidateData(validSamples, :);
        if isempty(candidateData)
            continue;
        end

        data = candidateData;
        selectedPath = candidatePath;
        turnRateCalibration = false;
        return;
    end

    error('plot_latest_log:NoCompatibleLog', ...
        ['No compatible format 4 log with at least %d columns and valid ', ...
         'tracking samples was found in %s.'], ...
        requiredColumnCount, logDirectory);
end
