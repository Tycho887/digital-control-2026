%% Editable experiment settings
Ts = 0.001;
Kp = 0.05;
referenceSpeed = 0.1;
stepTime = 0.2;
baselineDuration = 0.6;
runSweep = true;
sampleTimes = [0.001 0.002 0.005 0.01 0.02 0.03 0.04 0.05];
sweepDuration = 2;

%% Load model and run the baseline without changing saved model defaults
folder = fileparts(mfilename('fullpath'));
model = 'basebot_model_sampling';
if ~isfile(fullfile(folder,[model '.slx']))
    build_sampling_model();
end
load_system(fullfile(folder,[model '.slx']));
assert(Ts > 0 && all(sampleTimes > 0),'Sample times must be positive.');
assert(baselineDuration > stepTime && sweepDuration > stepTime,...
    'Simulation durations must exceed stepTime.');
outputFolder = fullfile(folder,'sampling_results');
if ~isfolder(outputFolder), mkdir(outputFolder); end
input = Simulink.SimulationInput(model);
input = input.setVariable('Kp',Kp,'Workspace',model);
input = input.setVariable('referenceSpeed',referenceSpeed,'Workspace',model);
input = input.setVariable('stepTime',stepTime,'Workspace',model);
input = input.setVariable('Ts',Ts,'Workspace',model);
input = input.setModelParameter('StopTime',num2str(baselineDuration,17));
aaa = sim(input);
h = figure('Name','Baseline sampling response');
plot(aaa.velP.Time,aaa.velP.Data,'LineWidth',1.5);
legend('Reference (m/s)','Motor voltage / 10','Velocity (m/s)','Location','best');
xlabel('Time (s)'); ylabel('Velocity (m/s), scaled voltage'); grid on;
title(sprintf('Ts = %.4g s, Kp = %.4g; sensor delay = Ts',Ts,Kp));
saveas(h,fullfile(outputFolder,'baseline.png'));
baselineSettings = struct('Ts',Ts,'Kp',Kp,'referenceSpeed',referenceSpeed,...
    'stepTime',stepTime,'duration',baselineDuration);
save(fullfile(outputFolder,'baseline.mat'),'aaa','baselineSettings');

%% Compare sample times; sensor latency follows each sample time
if runSweep
    sweepOutputs = cell(numel(sampleTimes),1);
    metrics = zeros(numel(sampleTimes),5);
    h = figure('Name','Sampling sweep');
    tiledlayout(ceil(numel(sampleTimes)/2),2);
    for k = 1:numel(sampleTimes)
        trial = input.setVariable('Ts',sampleTimes(k),'Workspace',model);
        trial = trial.setModelParameter('StopTime',num2str(sweepDuration,17));
        result = sim(trial);
        sweepOutputs{k} = result;
        t = result.velP.Time;
        velocity = result.velP.Data(:,3);
        % Uniform resampling avoids solver-step bias in summary statistics.
        gridTime = linspace(0,sweepDuration,10001)';
        v = interp1(t,velocity,gridTime,'linear');
        voltage = interp1(result.appliedVoltage.Time,...
            result.appliedVoltage.Data,gridTime,'previous');
        tail = gridTime >= sweepDuration-0.2;
        afterStep = gridTime >= stepTime;
        metrics(k,:) = [sampleTimes(k),referenceSpeed-mean(v(tail)),...
            max(0,max(v(afterStep))-referenceSpeed),...
            max(v(tail))-min(v(tail)),mean(abs(voltage(afterStep))>=9-1e-8)];
        nexttile;
        plot(t,result.velP.Data(:,[1 3]),'LineWidth',1.2); grid on;
        title(sprintf('Ts = %.4g s',sampleTimes(k)));
        xlabel('Time (s)'); ylabel('Velocity (m/s)');
    end
    summary = array2table(metrics,'VariableNames',{'Ts_s','SteadyStateError_mps',...
        'Overshoot_mps','FinalWindowVariation_mps','SaturationFraction'});
    disp(summary);
    saveas(h,fullfile(outputFolder,'sampling_sweep.png'));
    sweepSettings = struct('sampleTimes',sampleTimes,'Kp',Kp,...
        'referenceSpeed',referenceSpeed,'stepTime',stepTime,'duration',sweepDuration);
    save(fullfile(outputFolder,'sampling_sweep.mat'),'sweepOutputs','summary','sweepSettings');
    writetable(summary,fullfile(outputFolder,'sampling_summary.csv'));
end
