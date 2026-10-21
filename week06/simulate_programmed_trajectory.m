% Simulate the firmware's forward-turn-forward trajectory using the wheel model.
% Run from any folder with run('.../week06/simulate_programmed_trajectory.m').
folder = fileparts(mfilename('fullpath'));
addpath(folder);
[wheels, robot] = robot_model_defaults();

% Editable settings matching TrajectoryConfig in src/src/trajectory.h.
trajectory = struct('firstDistance',0.5,'turnAngle',pi/2, ...
    'secondDistance',0.5,'speed',0.15,'turnRate',0.50, ...
    'initialRest',4,'betweenRest',0.5,'finalRest',1);
values = cell2mat(struct2cell(trajectory));
assert(all(isfinite(values)) && trajectory.firstDistance>0 ...
    && trajectory.secondDistance>0 && trajectory.speed>0 ...
    && trajectory.turnRate>0 && abs(trajectory.turnAngle)>0 ...
    && trajectory.initialRest>=0 && trajectory.betweenRest>=0 ...
    && trajectory.finalRest>0, 'Invalid trajectory settings.');
durations = [trajectory.initialRest; trajectory.firstDistance/trajectory.speed; ...
    trajectory.betweenRest; abs(trajectory.turnAngle)/trajectory.turnRate; ...
    trajectory.betweenRest; trajectory.secondDistance/trajectory.speed; ...
    trajectory.finalRest];
boundaries = [0; cumsum(durations)];
commands = [0 0; trajectory.speed 0; 0 0; ...
    0 sign(trajectory.turnAngle)*trajectory.turnRate; 0 0; ...
    trajectory.speed 0; 0 0; 0 0];
% Zero-length rests are allowed: the last command at each boundary wins.
[~, keep] = unique(boundaries,'last');
commandSchedule = [boundaries(keep) commands(keep,:)];
robot.stopTime = boundaries(end);
robot.maxStep = min([wheels.Ts])/10;

model = 'week06_robot_model';
load_system(fullfile(folder,[model '.slx']));
inputs = Simulink.SimulationData.Dataset;
for k = 1:2
    signal = timeseries(commandSchedule(:,k+1),commandSchedule(:,1));
    inputs = inputs.addElement(setinterpmethod(signal,'zoh'),sprintf('command%d',k));
end
% SimulationInput applies overrides temporarily; the saved model is unchanged.
input = Simulink.SimulationInput(model);
input = input.setVariable('wheels',wheels,'Workspace',model);
input = input.setVariable('robot',robot,'Workspace',model);
input = input.setModelParameter('StopTime','robot.stopTime', ...
    'RelTol','robot.relativeTolerance','AbsTol','robot.absoluteTolerance', ...
    'MaxStep','robot.maxStep','ReturnWorkspaceOutputs','on');
for name = {'Velocity reference','Turn rate reference'}
    input = input.setBlockParameter([model '/' name{1}],'Interpolate','off');
end
input = input.setExternalInput(inputs);
out = sim(input);
response = out.robotOutputs;
assert(size(response.Data,2)==4 && all(isfinite(response.Data),'all'), ...
    'Expected finite [distance, yaw, x, y] model outputs.');
assert(abs(response.Time(1))<1e-9 ...
    && abs(response.Time(end)-robot.stopTime)<1e-9, ...
    'Model output does not cover the complete trajectory.');

% Include every command boundary as well as uniformly spaced plot samples.
time = unique([(0:min([wheels.Ts]):robot.stopTime)'; commandSchedule(:,1)]);
actual = interp1(response.Time,response.Data,time);
reference = interp1(commandSchedule(:,1),commandSchedule(:,2:3),time,'previous');
% Exact integration of ideal held commands, in model order [distance, yaw, x, y].
ideal = zeros(numel(time),4);
state = [robot.initialDistance robot.initialAngle robot.initialX robot.initialY];
for n = 1:numel(time)
    ideal(n,:) = state;
    if n == numel(time), break; end
    dt = time(n+1)-time(n);
    v = reference(n,1); w = reference(n,2);
    theta = state(2)-robot.initialAngle;
    if abs(w)<1e-12
        delta = v*dt*[cos(theta) sin(theta)];
    else
        delta = v/w*[sin(theta+w*dt)-sin(theta), cos(theta)-cos(theta+w*dt)];
    end
    state = state+[v*dt w*dt delta];
end

fig = figure('Name','Programmed robot trajectory','Position',[100 100 1100 750], ...
    'Theme','light');
layout = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
columns = [3 4 1 2];
titles = {'x position','y position','Accumulated forward distance','Total yaw'};
labels = {'x (m)','y (m)','Signed distance (m)','Unwrapped yaw (rad)'};
for k = 1:4
    nexttile;
    plot(time,actual(:,columns(k)),'LineWidth',1.5);
    hold on;
    plot(time,ideal(:,columns(k)),'--','LineWidth',1.2);
    grid on; xlim([0 robot.stopTime]);
    title(titles{k}); xlabel('Time (s)'); ylabel(labels{k});
    legend('Simulink wheel model','Ideal commands','Location','best');
end
title(layout,sprintf('Forward %.2f m, turn %.0f deg, forward %.2f m', ...
    trajectory.firstDistance,rad2deg(trajectory.turnAngle),trajectory.secondDistance));
resultsFolder = fullfile(folder,'robot_results');
if ~isfolder(resultsFolder), mkdir(resultsFolder); end
exportgraphics(fig,fullfile(resultsFolder,'programmed_trajectory.png'),'Resolution',180);
outputColumns = {'distance_m','yaw_rad','x_m','y_m'};
save(fullfile(resultsFolder,'programmed_trajectory.mat'),'out','wheels','robot', ...
    'trajectory','commandSchedule','time','actual','ideal','outputColumns');
fprintf('Simulated %.6f s. Final x=%.6f m, y=%.6f m, distance=%.6f m, yaw=%.6f rad.\n', ...
    robot.stopTime,actual(end,3),actual(end,4),actual(end,1),actual(end,2));
fprintf('Plot and data saved in %s\n',resultsFolder);
