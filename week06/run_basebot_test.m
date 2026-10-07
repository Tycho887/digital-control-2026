function summary = run_basebot_test()
% Two measured motors, each with its own sampled P speed controller.
% Run from any folder: addpath('week06'); run_basebot_test
folder = fileparts(mfilename('fullpath'));
% Editable experiment settings (Kp is V/(rad/s)).
Kp = [0.05 0.05];
useTunedGains = true; % Set false to use the manual gains above.
Ts = 0.001;
referenceSpeed = [0.1 0.1]; % left, right, m/s
stepTime = 0.2;
stopTime = 3;
steadyWindow = 0.5;

% Reuse the root analysis: the 6 V step at 0.020–0.500 s in this log.
% It uses 0.927 kg robot mass and motor-side reflected inertia.
run(fullfile(folder,'..','analysis.m'));
measured = [left right];
wheels = repmat(struct('L',0,'R',0,'Kt',0,'Kb',0,'J',0,'D',0, ...
    'gear',9.68,'wheelRadius',0.03,'Kp',0,'Ts',Ts,'voltageLimit',9),1,2);
for k=1:2
    for name={'L','R','Kb','J','D'}
        wheels(k).(name{1})=measured(k).(name{1});
    end
    wheels(k).Kt=measured(k).Kb; % SI assumption, as in the analysis
    wheels(k).Kp=Kp(k);
    assert(all(structfun(@(v)isfinite(v)&&v>0,wheels(k))), ...
        'Motor parameters must be finite and positive.');
end
gainFile=fullfile(folder,'basebot_p_gains.mat');
if useTunedGains && isfile(gainFile)
    tuned=load(gainFile,'Kp','motorParameters');
    for k=1:2
        check=wheels(k); check.Kp=tuned.motorParameters(k).Kp;
        assert(isequal(check,tuned.motorParameters(k)), ...
            'Motor parameters or Ts changed. Rerun tune_basebot_p.');
        wheels(k).Kp=tuned.Kp(k);
    end
end
parameters=struct2table(wheels);
parameters.Wheel=["left";"right"];
writetable(parameters,fullfile(folder,'basebot_motor_parameters.csv'));

model='week06_basebot_test';
target=fullfile(folder,[model '.slx']);
if ~isfile(target)
    buildModel(model,target,folder);
else
    load_system(target);
end
workspace=get_param(model,'ModelWorkspace');
assignin(workspace,'wheels',wheels);
assignin(workspace,'referenceSpeed',referenceSpeed);
assignin(workspace,'stepTime',stepTime);
set_param(model,'StopTime',num2str(stopTime),'MaxStep','min([wheels.Ts])/10');
save_system(model,target);
out=sim(model);
time=linspace(stopTime-steadyWindow,stopTime,5001)';
speed=zeros(2,1); variation=speed; expected=speed; saturated=speed;
signals={out.leftDiagnostics,out.rightDiagnostics};
for k=1:2
    p=wheels(k);
    values=interp1(signals{k}.Time,signals{k}.Data,time);
    speed(k)=mean(values(:,4));
    variation(k)=max(values(:,4))-min(values(:,4));
    saturated(k)=mean(abs(values(:,2)*10)>=p.voltageLimit-1e-6);
    loss=p.Kb+p.R*p.D/p.Kt;
    expected(k)=referenceSpeed(k)*p.Kp/(p.Kp+loss);
    assert(all(isfinite(signals{k}.Data),'all'));
    assert(max(abs(signals{k}.Data(:,2)*10))<=p.voltageLimit+1e-8);
end
settled=variation<1e-5;
error=referenceSpeed(:)-speed;
summary=table(["left";"right"],referenceSpeed(:),speed,error, ...
    100*error./referenceSpeed(:),expected,variation,saturated,settled, ...
    'VariableNames',{'Wheel','Reference_mps','MeanSpeed_mps','Error_mps', ...
    'Error_percent','PredictedSpeed_mps','FinalVariation_mps', ...
    'SaturationFraction','Settled'});
assert(all(settled),'Response has not settled: increase stopTime or check Kp/Ts.');
assert(all(abs(speed-expected)<1e-5), ...
    'Simulation differs from unsaturated steady-state prediction.');
writetable(summary,fullfile(folder,'basebot_steady_state.csv'));
save(fullfile(folder,'basebot_test_results.mat'),'out','wheels','summary','logPath');
fig=figure('Visible','off');
cleanup=onCleanup(@()close(fig));
tiledlayout(2,1);
nexttile; hold on;
plot(out.leftDiagnostics.Time,out.leftDiagnostics.Data(:,4),'LineWidth',1.5);
plot(out.rightDiagnostics.Time,out.rightDiagnostics.Data(:,4),'LineWidth',1.5);
plot(out.leftDiagnostics.Time,out.leftDiagnostics.Data(:,1),'--');
plot(out.rightDiagnostics.Time,out.rightDiagnostics.Data(:,1),':');
grid on; ylabel('Speed (m/s)'); legend('Left','Right','Left reference','Right reference');
title('Basebot: measured motors with independent P controllers');
nexttile; hold on;
for k=1:2
    plot(signals{k}.Time,signals{k}.Data(:,1)-signals{k}.Data(:,4),'LineWidth',1.5);
end
grid on; ylabel('Reference - speed (m/s)'); xlabel('Time (s)'); legend('Left','Right');
exportgraphics(fig,fullfile(folder,'basebot_steady_state.png'),'Resolution',180);
disp(summary);
fprintf('Source: %s\nResults saved in %s\n',logPath,folder);
end

function buildModel(model,target,folder)
load_system(fullfile(folder,'week06_wheel_model.slx'));
new_system(model);
set_param(model,'Solver','ode45','SolverType','Variable-step', ...
    'RelTol','1e-6','AbsTol','1e-9','ReturnWorkspaceOutputs','on');
for k=1:2
    names={'Left','Right'}; name=names{k}; y=80+160*(k-1);
    block=[model '/' name ' wheel'];
    add_block('week06_wheel_model/Wheel',block,'Position',[230 y 450 y+70]);
    fields={'L','R','Kt','Kb','J','D','gear','wheelRadius','Kp','Ts','voltageLimit'};
    for j=1:numel(fields)
        set_param(block,fields{j},sprintf('wheels(%d).%s',k,fields{j}));
    end
    add_block('simulink/Sources/Step',[model '/' name ' reference'], ...
        'Position',[60 y+15 90 y+45],'Time','stepTime', ...
        'After',sprintf('referenceSpeed(%d)',k),'SampleTime','0');
    add_block('simulink/Ports & Subsystems/Out1',[model '/' name ' velocity'], ...
        'Position',[530 y 560 y+20],'Port',num2str(k));
    add_block('simulink/Sinks/To Workspace',[model '/' name ' diagnostics'], ...
        'Position',[520 y+45 660 y+75],'SaveFormat','Timeseries', ...
        'MaxDataPoints','inf','VariableName',[lower(name) 'Diagnostics']);
    add_line(model,[name ' reference/1'],[name ' wheel/1']);
    add_line(model,[name ' wheel/1'],[name ' velocity/1']);
    add_line(model,[name ' wheel/2'],[name ' diagnostics/1']);
end
% The caller installs measured parameters and saves the model.
end
