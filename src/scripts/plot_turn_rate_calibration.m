function [figureHandle, pngPath, capture] = plot_turn_rate_calibration(logPath, outputDirectory)
% Plot one automatically recorded start-button calibration capture.
capture = read_turn_rate_calibration(logPath);
data = capture.data;
time = data(:,1);
if nargin < 2, outputDirectory = fileparts(logPath); end
[~, name] = fileparts(logPath);
figureHandle = figure('Color','w', 'Name','Turn-rate calibration sequence', ...
    'NumberTitle','off', 'Position',[100,100,1280,1000]);
layout = tiledlayout(figureHandle,4,1,'TileSpacing','compact','Padding','compact');
sgtitle(layout, ['Turn-rate calibration sequence: ',name,' (',capture.result,')'], ...
    'Interpreter','none','Color','k');
axesHandles = gobjects(4,1);
palette = [0,.447,.741; .85,.325,.098; .466,.674,.188; ...
    .494,.184,.556; .301,.745,.933; .635,.078,.184; .929,.694,.125];

axesHandles(1) = nexttile(layout);
colororder(axesHandles(1),palette);
plot(time, data(:,46), 'k--', 'DisplayName','Wheel yaw reference'); hold on;
plot(time, data(:,49), 'DisplayName','Mapped gyro');
plot(time, data(:,51), 'DisplayName','Wheel measurement');
plot(time, data(:,29), 'DisplayName','Fused estimate'); hold off;
ylabel('Yaw rate (rad/s)','Color','k');
title('Reference, gyro, wheels, and estimator','Color','k');

axesHandles(2) = nexttile(layout);
colororder(axesHandles(2),palette);
plot(time, deg2rad(data(:,13:15)), 'LineWidth',.8);
ylabel('Gyro axes (rad/s)','Color','k');
title('All reported gyro axes after offset removal','Color','k');
legend({'X','Y','Z'}, 'Location','best','TextColor','k','Color','w');

axesHandles(3) = nexttile(layout);
colororder(axesHandles(3),palette);
plot(time,rad2deg(capture.gyroAngle), 'LineWidth',.8); hold on;
plot(time,rad2deg(capture.mappedAngle), 'k-', 'LineWidth',1.3);
plot(time,rad2deg(capture.wheelAngle), '--', 'LineWidth',1.2);
plot(time,rad2deg(capture.fusedAngle), ':', 'LineWidth',1.2); hold off;
ylabel('Angle (degrees)','Color','k');
title('Unwrapped angles; gyro integrals use every fresh sensor sample','Color','k');
legend({'Gyro X','Gyro Y','Gyro Z','Mapped gyro','Encoder counts','Fused'}, ...
    'Location','best','NumColumns',3,'TextColor','k','Color','w');

axesHandles(4) = nexttile(layout);
colororder(axesHandles(4),palette);
plot(time,data(:,5:6),'LineWidth',1.1); hold on;
plot(time,data(:,7:8),'--','LineWidth',.8); hold off;
ylabel('Motor speed (rad/s)','Color','k'); xlabel('Time (s)','Color','k');
title('Wheel PI tracking; motors are off during calibration and pauses','Color','k');
legend({'Left reference','Right reference','Left feedback','Right feedback'}, ...
    'Location','best','NumColumns',2,'TextColor','k','Color','w');

for axis = axesHandles'
    hold(axis,'on');
    xline(axis,4,':','Calibration complete','Color',[.4,.4,.4], ...
        'LabelVerticalAlignment','bottom','HandleVisibility','off');
    hold(axis,'off');
    grid(axis,'on');
end
legend(axesHandles(1),'Location','best','NumColumns',2,'TextColor','k','Color','w');
linkaxes(axesHandles,'x');
set(axesHandles,'Color','w','XColor','k','YColor','k', ...
    'GridColor',[.25,.25,.25],'GridAlpha',.18);
pngPath = fullfile(outputDirectory,[name,'_turn_rate_calibration.png']);
exportgraphics(figureHandle,pngPath,'Resolution',160);
fprintf('Turn-rate calibration sequence: %s\nResult: %s\n',logPath,capture.result);
fprintf('Gravity (library g): %s\n',mat2str(capture.gravity,5));
fprintf('Hardware count/library conversion max error: %.6g deg/s\n',capture.conversionError);
disp(capture.summary);
for axis = 1:3
    fprintf('Gyro %s = %.5f * wheel yaw + %+.5f rad/s\n', ...
        char('X'+axis-1),capture.fit(axis,1),capture.fit(axis,2));
end
fprintf('Upside-down wheel motion is not body yaw. Saved %s\n',pngPath);
end
