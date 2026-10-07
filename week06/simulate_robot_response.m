% Editable robot command experiment and nominal wheel-loop stability report.
% Run from any folder with run('.../week06/simulate_robot_response.m').
folder = fileparts(mfilename('fullpath'));
addpath(folder);
[wheels, robot] = robot_model_defaults();
% Override any wheel/robot parameters here before configuring the model.
% Each row starts a held command: [time_s, velocity_mps, turn_rate_radps].
commandSchedule = [0 0 0; 0.5 0.1 0; 3.5 0 0.5; 6.5 0.1 0.5; ...
    9.5 0.1 -0.5; 12.5 -0.1 0; 15.5 0 0; 20 0 0];
robot.stopTime = commandSchedule(end,1);
robot.maxStep = min([wheels.Ts])/10;
model = configure_robot_model(wheels,robot);
assert(size(commandSchedule,2)==3 && all(isfinite(commandSchedule),'all') ...
    && commandSchedule(1,1)==0 && all(diff(commandSchedule(:,1))>0), ...
    'Use finite [time, velocity, turn rate] rows with increasing times from zero.');
% Root Inports otherwise interpolate between command samples.
for name = {'Velocity reference','Turn rate reference'}
    set_param([model '/' name{1}],'Interpolate','off');
end
inputs = Simulink.SimulationData.Dataset;
for k = 1:2
    signal = timeseries(commandSchedule(:,k+1),commandSchedule(:,1));
    signal = setinterpmethod(signal,'zoh');
    inputs = inputs.addElement(signal,sprintf('command%d',k));
end
% Log the existing wheel diagnostic ports without changing the saved model.
wheelNames = {'Left','Right'};
for k = 1:2
    ports = get_param([model '/Robot/' wheelNames{k} ' wheel'],'PortHandles');
    set_param(ports.Outport(2),'DataLogging','on','DataLoggingNameMode','Custom', ...
        'DataLoggingName',[lower(wheelNames{k}) 'Diagnostics']);
end
input = Simulink.SimulationInput(model);
input = input.setExternalInput(inputs);
input = input.setModelParameter('SignalLogging','on','SignalLoggingName','logsout');
out = sim(input);
response = out.robotOutputs;
assert(all(isfinite(response.Data),'all'),'Non-finite robot response.');
% Uniform sampling avoids weighting solver-dense transients more heavily.
time = (0:min([wheels.Ts]):robot.stopTime)';
actual = interp1(response.Time,response.Data,time);
reference = interp1(commandSchedule(:,1),commandSchedule(:,2:3),time,'previous');
wheelReference = [reference(:,1)-robot.trackWidth*reference(:,2)/2, ...
    reference(:,1)+robot.trackWidth*reference(:,2)/2];
wheelActual = [actual(:,2)-robot.trackWidth*actual(:,4)/2, ...
    actual(:,2)+robot.trackWidth*actual(:,4)/2];
voltage = zeros(numel(time),2);
for k = 1:2
    diagnostics = out.logsout.get([lower(wheelNames{k}) 'Diagnostics']).Values;
    voltage(:,k) = interp1(diagnostics.Time,diagnostics.Data(:,2),time) ...
        /wheels(k).diagnosticVoltageScale;
    assert(max(abs(voltage(:,k)))<=wheels(k).voltageLimit+1e-6, ...
        'Wheel voltage exceeds its configured limit.');
end
% The steering/output maps add no feedback between the two speed loops.
% This linear check uses the same ZOH + one-sample latency approximation
% as week06 tuning. The actual nonlinear model above has transport delay.
stability = table();
for k = 1:2
    p = wheels(k);
    plant = tf(p.Kt,[p.L*p.J p.L*p.D+p.R*p.J p.R*p.D+p.Kt*p.Kb]);
    digital = c2d(plant,p.Ts,'zoh'); digital.InputDelay = 1;
    closedLoop = feedback(delay2z(ss(p.Kp*digital)),1);
    poles = pole(closedLoop);
    [gm,pm] = margin(p.Kp*digital);
    row = table(string(wheelNames{k}),p.Kp,p.Ts,max(abs(poles)), ...
        isstable(closedLoop),20*log10(gm),pm, ...
        'VariableNames',{'Wheel','Kp','Ts_s','MaxPoleMagnitude', ...
        'LinearStable','GainMargin_dB','PhaseMargin_deg'});
    stability = [stability; row]; %#ok<AGROW>
end
% Report final 0.5 s of each held segment, including residual straight drift.
segments = table();
for k = 1:size(commandSchedule,1)-1
    startTime = commandSchedule(k,1); endTime = commandSchedule(k+1,1);
    tail = time>=max(startTime,endTime-0.5) & time<endTime;
    meanWheel = mean(wheelActual(tail,:),1);
    variation = max(wheelActual(tail,:),[],1)-min(wheelActual(tail,:),[],1);
    ref = commandSchedule(k,2:3);
    refWheel = [ref(1)-robot.trackWidth*ref(2)/2,ref(1)+robot.trackWidth*ref(2)/2];
    loss = [wheels.Kb]+[wheels.R].*[wheels.D]./[wheels.Kt];
    predictedWheel = refWheel.*[wheels.Kp]./([wheels.Kp]+loss);
    row = table(startTime,endTime,ref(1),ref(2),mean(actual(tail,2)), ...
        mean(actual(tail,4)),variation(1),variation(2), ...
        max(abs(meanWheel-predictedWheel)),all(variation<1e-5), ...
        'VariableNames',{'Start_s','End_s','VelocityRef_mps','TurnRateRef_radps', ...
        'MeanVelocity_mps','MeanTurnRate_radps','LeftVariation_mps', ...
        'RightVariation_mps','PredictionError_mps','Settled'});
    segments = [segments;row]; %#ok<AGROW>
end
resultsFolder = fullfile(folder,'robot_results');
if ~isfolder(resultsFolder), mkdir(resultsFolder); end
writetable(stability,fullfile(resultsFolder,'stability.csv'));
writetable(segments,fullfile(resultsFolder,'segments.csv'));
save(fullfile(resultsFolder,'response.mat'),'out','wheels','robot', ...
    'commandSchedule','stability','segments');
fig = figure('Name','Two-wheel P controller response');
tiledlayout(3,2);
nexttile; plot(time,[reference(:,1) actual(:,2)]); grid on;
ylabel('Velocity (m/s)'); legend('Command','Actual');
nexttile; plot(time,[reference(:,2) actual(:,4)]); grid on;
ylabel('Turn rate (rad/s)'); legend('Command','Actual');
nexttile; plot(time,[wheelReference wheelActual]); grid on;
ylabel('Wheel speed (m/s)'); legend('Left command','Right command','Left actual','Right actual');
nexttile; plot(time,voltage); grid on;
ylabel('Motor voltage (V)'); legend('Left','Right');
nexttile; plot(time,actual(:,1)); grid on; ylabel('Signed distance (m)'); xlabel('Time (s)');
nexttile; plot(time,actual(:,3)); grid on; ylabel('Turn angle (rad)'); xlabel('Time (s)');
exportgraphics(fig,fullfile(resultsFolder,'response.png'),'Resolution',180);
disp(stability); disp(segments);
fprintf('Results: %s\n',resultsFolder);
fprintf('Linear speed loops stable: %d. All held segments settled: %d.\n', ...
    all(stability.LinearStable),all(segments.Settled));
fprintf(['Distance and heading integrate motion: they need not remain bounded under\n' ...
    'constant commands or residual wheel mismatch. This is not pose regulation.\n']);
