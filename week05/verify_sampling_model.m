function verify_sampling_model()
% Validate wiring, plant equivalence and closed-loop limits with simulations.
folder = fileparts(mfilename('fullpath'));
model = 'basebot_model_sampling';
load_system(fullfile(folder,[model '.slx']));
assert(strcmp(get_param([model '/DA hold'],'SampleTime'),'Ts'));
assert(strcmp(get_param([model '/AD hold'],'SampleTime'),'Ts'));
assert(strcmp(get_param([model '/detect_delay'],'DelayTime'),'Ts'));
input = Simulink.SimulationInput(model);
out = sim(input.setModelParameter('StopTime','0.6'));
assert(size(out.velP.Data,2)==3);
assert(all(isfinite(out.velP.Data),'all'));
assert(max(abs(out.appliedVoltage.Data)) <= 9+1e-9);
assert(abs(out.velP.Data(end,3)-0.1*0.05/(0.05+0.011)) < 1e-4);

% Drive a temporary copy of the Motor subsystem with the original sequence.
testModel = 'sampling_plant_verification';
new_system(testModel);
cleanup = onCleanup(@() close_system(testModel,0));
add_block([model '/Motor'],[testModel '/Motor']);
add_block('simulink/Sources/From Workspace',[testModel '/Voltage'],...
    'VariableName','voltageSequence','SampleTime','0.0005','Interpolate','off',...
    'OutputAfterFinalValue','Setting to zero');
add_block('simulink/Sinks/To Workspace',[testModel '/Omega'],...
    'VariableName','omega','SaveFormat','Timeseries');
add_block('simulink/Sinks/To Workspace',[testModel '/Current'],...
    'VariableName','current','SaveFormat','Timeseries');
add_line(testModel,'Voltage/1','Motor/1');
add_line(testModel,'Motor/1','Omega/1');
add_line(testModel,'Motor/2','Current/1');
set_param(testModel,'Solver','ode45','RelTol','1e-8','AbsTol','1e-10',...
    'MaxStep','0.0001','ReturnWorkspaceOutputs','on');
seq = [0 0 0;0.02 1.5 1.5;0.5 3 3;1 6 6;1.5 0 0;2 0 0];
parameters = struct('L',1e-3,'R',1,'Kt',0.01,'Kb',0.01,'J',1e-5,...
    'D',1e-5,'gear',9.68,'wheelRadius',0.03,'r2m',0.03/9.68,'seq',seq);
load_system(fullfile(folder,'motor_model_23b.slx'));
original = Simulink.SimulationInput('motor_model_23b');
copy = Simulink.SimulationInput(testModel);
names = fieldnames(parameters);
for k = 1:numel(names)
    original = original.setVariable(names{k},parameters.(names{k}));
    copy = copy.setVariable(names{k},parameters.(names{k}));
end
original = original.setModelParameter('StopTime','2','Solver','ode45',...
    'RelTol','1e-8','AbsTol','1e-10','MaxStep','0.0001');
copy = copy.setVariable('voltageSequence',seq(:,[1 2]));
a = sim(original);
b = sim(copy.setModelParameter('StopTime','2'));
times = linspace(0,2,20001)';
originalValues = interp1(a.so.Time,a.so.Data(:,[4 2]),times);
copyValues = [interp1(b.omega.Time,b.omega.Data,times),...
    interp1(b.current.Time,b.current.Data,times)];
error = max(abs(originalValues-copyValues),[],1);
assert(error(1)<1e-3 && error(2)<1e-4,'Copied motor differs from original.');
fprintf('Validation passed. Baseline final velocity %.8f m/s. Plant errors: omega %.3g rad/s, current %.3g A.\n',...
    out.velP.Data(end,3),error(1),error(2));
end
