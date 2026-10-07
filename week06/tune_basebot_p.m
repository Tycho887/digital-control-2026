function selected = tune_basebot_p()
% P-only pidtune sweep, validated in the nonlinear sampled Simulink model.
% Objective: minimum integral absolute speed error among candidates with
% <2% steady error, settled response, and <15% reference overshoot.
folder=fileparts(mfilename('fullpath'));
model='week06_basebot_test';
load_system(fullfile(folder,[model '.slx']));
workspace=get_param(model,'ModelWorkspace');
motorParameters=workspace.evalin('wheels');
rows=table();
frequencies=logspace(log10(20),log10(1000),24); % rad/s crossover requests
options=pidtuneOptions('PhaseMargin',60);
for n=1:numel(frequencies)
    candidates=motorParameters;
    stable=false(1,2); gains=zeros(1,2);
    for k=1:2
        p=motorParameters(k);
        plant=tf(p.Kt,[p.L*p.J p.L*p.D+p.R*p.J p.R*p.D+p.Kt*p.Kb]);
        % DA zero-order hold and AD measurement with one sample latency.
        digital=c2d(plant,p.Ts,'zoh');
        digital.InputDelay=1;
        [controller,info]=pidtune(digital,'P',frequencies(n),options);
        gains(k)=controller.Kp;
        stable(k)=info.Stable && isfinite(gains(k)) && gains(k)>0;
        candidates(k).Kp=gains(k);
    end
    if ~all(stable), continue; end
    input=Simulink.SimulationInput(model);
    input=input.setVariable('wheels',candidates,'Workspace',model);
    input=input.setVariable('referenceSpeed',[0.1 0.1],'Workspace',model);
    input=input.setVariable('stepTime',0.2,'Workspace',model);
    out=sim(input.setModelParameter('StopTime','3'));
    signals={out.leftDiagnostics,out.rightDiagnostics};
    for k=1:2
        signal=signals{k};
        t=linspace(0.2,3,14001)';
        v=interp1(signal.Time,signal.Data(:,4),t);
        tail=v(t>=2.5);
        errorPercent=100*abs(0.1-mean(tail))/0.1;
        variation=max(tail)-min(tail);
        overshoot=100*max(0,max(v)-0.1)/0.1;
        iae=trapz(t,abs(0.1-v));
        feasible=all(isfinite(v)) && errorPercent<2 && variation<1e-5 && overshoot<15;
        row=table(k,frequencies(n),gains(k),errorPercent,overshoot,variation,iae,feasible, ...
            'VariableNames',{'Motor','RequestedCrossover_radps','Kp','Error_percent', ...
            'Overshoot_percent','FinalVariation_mps','IAE_m','Feasible'});
        rows=[rows;row]; %#ok<AGROW>
    end
end
selected=table(); Kp=zeros(1,2);
for k=1:2
    valid=rows(rows.Motor==k & rows.Feasible,:);
    assert(~isempty(valid),'No feasible P gain found for motor %d. Inspect sweep.',k);
    [~,best]=min(valid.IAE_m);
    selected=[selected;valid(best,:)]; %#ok<AGROW>
    Kp(k)=valid.Kp(best);
end
% Write only after both motors have a verified feasible candidate.
save(fullfile(folder,'basebot_p_gains.mat'),'Kp','motorParameters','selected','options','frequencies');
writetable(rows,fullfile(folder,'basebot_p_tuning_sweep.csv'));
writetable(selected,fullfile(folder,'basebot_p_selected.csv'));
for k=1:2, motorParameters(k).Kp=Kp(k); end
assignin(workspace,'wheels',motorParameters);
save_system(model);
disp(selected);
fprintf('Saved P gains: left %.8g, right %.8g V/(rad/s).\n',Kp);
% Refresh the normal Basebot results using the saved gains.
run_basebot_test();
close_system(model,0);
end
