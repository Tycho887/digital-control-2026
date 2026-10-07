function verify_week06()
% Check clean-session defaults, script overrides and independent wheel copies.
folder=fileparts(mfilename('fullpath'));
model='week06_wheel_model';
load_system(fullfile(folder,[model '.slx']));
a=sim(model);
assert(all(isfinite(a.velP.Data),'all'));
assert(size(a.velP.Data,2)==5);
assert(max(abs(a.velP.Data(:,2)*10))<=9+1e-9);
assert(abs(a.velP.Data(end,4)-0.1*0.05/(0.05+0.011))<1e-5);
run(fullfile(folder,'init_week06.m'));
input=Simulink.SimulationInput(model);
input=input.setVariable('referenceSpeed',-0.1,'Workspace',model);
b=sim(input);
assert(abs(b.velP.Data(end,4)+a.velP.Data(end,4))<1e-5);
% Compare all five channels with week05 (its logger has only three).
load_system(fullfile(folder,'..','week05','basebot_model_sampling.slx'));
c=sim('basebot_model_sampling');
t=linspace(0,0.6,6001)';
x=interp1(a.velP.Time,a.velP.Data,t);
y=[interp1(c.velP.Time,c.velP.Data(:,1:2),t), ...
    interp1(c.omega.Time,c.omega.Data,t), ...
    interp1(c.velP.Time,c.velP.Data(:,3),t), ...
    interp1(c.motorCurrent.Time,c.motorCurrent.Data,t)];
errors=max(abs(x-y),[],1);
assert(all(errors<[1e-8 1e-4 1e-2 1e-4 1e-3]));
test='week06_two_wheels_verification';
new_system(test);
cleanup=onCleanup(@()close_system(test,0));
set_param(test,'Solver','ode45','RelTol','1e-6','AbsTol','1e-9', ...
    'MaxStep','0.0001','StopTime','0.6','ReturnWorkspaceOutputs','on');
for k=1:2
    name=sprintf('Wheel%d',k);
    add_block([model '/Wheel'],[test '/' name]);
    p=wheel;
    if k==2, p.Kp=0.1; p.R=1.2; end
    fields=fieldnames(p);
    for j=1:numel(fields)
        set_param([test '/' name],fields{j},num2str(p.(fields{j}),17));
    end
    add_block('simulink/Sources/Step',[test '/Ref' num2str(k)], ...
        'Time','0.2','After',num2str((-1)^(k+1)*0.1));
    add_block('simulink/Sinks/To Workspace',[test '/Log' num2str(k)], ...
        'VariableName',['v' num2str(k)],'SaveFormat','Timeseries');
    add_block('simulink/Sinks/Terminator',[test '/Unused' num2str(k)]);
    add_line(test,['Ref' num2str(k) '/1'],[name '/1']);
    add_line(test,[name '/1'],['Log' num2str(k) '/1']);
    add_line(test,[name '/2'],['Unused' num2str(k) '/1']);
end
d=sim(test);
assert(abs(d.v1.Data(end)-a.velP.Data(end,4))<1e-5);
assert(abs(d.v2.Data(end)+0.1*0.1/(0.1+0.01+1.2*1e-5/0.01))<1e-5);
fprintf('PASS: defaults, script initialization, reverse reference, week05 equivalence and independent wheel copies.\n');
fprintf('Baseline: %.8f m/s; week05 channel errors: %s\n',a.velP.Data(end,4),mat2str(errors,3));
close_system(model,0);
close_system('basebot_model_sampling',0);
end
