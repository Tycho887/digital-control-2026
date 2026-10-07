function model = build_robot_model()
% Reproduce the composite model without identification, tuning or simulation.
folder = fileparts(mfilename('fullpath'));
model = 'week06_robot_model';
assert(~bdIsLoaded(model),'Close week06_robot_model before rebuilding it.');
source = 'week06_basebot_test';
wasLoaded = bdIsLoaded(source);
load_system(fullfile(folder,[source '.slx']));
cleanup = onCleanup(@()closeSource(source,wasLoaded)); %#ok<NASGU>
new_system(model);
[wheels, robot] = robot_model_defaults();
workspace = get_param(model,'ModelWorkspace');
assignin(workspace,'wheels',wheels);
assignin(workspace,'robot',robot);
set_param(model,'Solver','ode45','SolverType','Variable-step', ...
    'ReturnWorkspaceOutputs','on','StopTime','robot.stopTime', ...
    'RelTol','robot.relativeTolerance','AbsTol','robot.absoluteTolerance', ...
    'MaxStep','robot.maxStep');
body = [model '/Robot'];
add_block('built-in/Subsystem',body,'Position',[230 80 460 240]);
mask = Simulink.Mask.create(body);
mask.addParameter('Type','edit','Name','wheelParameters','Prompt','Left/right wheel structures','Value','wheels');
mask.addParameter('Type','edit','Name','robotParameters','Prompt','Robot structure','Value','robot');
mask.Display = "disp('Two-wheel robot'); port_label('input',1,'velocity ref (m/s)'); port_label('input',2,'turn rate ref (rad/s)'); port_label('output',1,'distance (m)'); port_label('output',2,'velocity (m/s)'); port_label('output',3,'turn angle (rad)'); port_label('output',4,'turn rate (rad/s)');";
add_block('built-in/Inport',[body '/Velocity reference'],'Position',[30 65 60 85],'Port','1');
add_block('built-in/Inport',[body '/Turn rate reference'],'Position',[30 175 60 195],'Port','2');
add_block('built-in/Gain',[body '/Half track width'],'Position',[100 165 200 195],'Gain','robotParameters.trackWidth/2');
add_block('built-in/Sum',[body '/Left reference'],'Position',[250 60 280 90],'Inputs','+-');
add_block('built-in/Sum',[body '/Right reference'],'Position',[250 220 280 250],'Inputs','++');
add_line(body,'Velocity reference/1','Left reference/1');
add_line(body,'Velocity reference/1','Right reference/1');
add_line(body,'Turn rate reference/1','Half track width/1');
add_line(body,'Half track width/1','Left reference/2');
add_line(body,'Half track width/1','Right reference/2');
for k = 1:2
    names = {'Left','Right'}; name = names{k}; y = 40+160*(k-1);
    wheelBlock = [body '/' name ' wheel'];
    add_block([source '/' name ' wheel'],wheelBlock,'Position',[340 y 550 y+80]);
    fields = {'L','R','Kt','Kb','J','D','gear','wheelRadius','Kp','Ts','voltageLimit'};
    for j = 1:numel(fields)
        set_param(wheelBlock,fields{j},sprintf('wheelParameters(%d).%s',k,fields{j}));
    end
    wheelMask = Simulink.Mask.get(wheelBlock);
    fields = {'initialMotorSpeed','initialCurrent','initialSensorSpeed','diagnosticVoltageScale'};
    for j = 1:numel(fields)
        wheelMask.addParameter('Type','edit','Name',fields{j},'Prompt',fields{j}, ...
            'Value',sprintf('wheelParameters(%d).%s',k,fields{j}));
    end
    set_param([wheelBlock '/Motor/Integrator'],'InitialCondition','initialCurrent');
    set_param([wheelBlock '/Motor/Integrator1'],'InitialCondition','initialMotorSpeed');
    delays = find_system(wheelBlock,'LookUnderMasks','all','BlockType','TransportDelay');
    set_param(delays{1},'InitialOutput','initialSensorSpeed');
    set_param([wheelBlock '/Voltage plot scale'],'Gain','diagnosticVoltageScale');
    add_block('built-in/Terminator',[body '/' name ' diagnostics'],'Position',[590 y+55 610 y+75]);
    add_line(body,[name ' reference/1'],[name ' wheel/1']);
    add_line(body,[name ' wheel/2'],[name ' diagnostics/1']);
end
add_block('built-in/Sum',[body '/Wheel speed sum'],'Position',[650 50 680 80],'Inputs','++');
add_block('built-in/Sum',[body '/Wheel speed difference'],'Position',[650 210 680 240],'Inputs','-+');
for name = {'sum','difference'}
    add_line(body,'Left wheel/1',['Wheel speed ' name{1} '/1']);
    add_line(body,'Right wheel/1',['Wheel speed ' name{1} '/2']);
end
add_block('built-in/Gain',[body '/Average velocity'],'Position',[720 50 800 80],'Gain','1/2');
add_block('built-in/Gain',[body '/Turn rate'],'Position',[720 210 800 240],'Gain','1/robotParameters.trackWidth');
add_line(body,'Wheel speed sum/1','Average velocity/1');
add_line(body,'Wheel speed difference/1','Turn rate/1');
add_block('built-in/Integrator',[body '/Distance'],'Position',[850 30 890 60],'InitialCondition','robotParameters.initialDistance');
add_block('built-in/Integrator',[body '/Turn angle'],'Position',[850 190 890 220],'InitialCondition','robotParameters.initialAngle');
add_line(body,'Average velocity/1','Distance/1');
add_line(body,'Turn rate/1','Turn angle/1');
outputs = {'Distance','Velocity','Turn angle','Turn rate'};
sources = {'Distance','Average velocity','Turn angle','Turn rate'};
for k = 1:4
    y = 30+70*(k-1);
    add_block('built-in/Outport',[body '/' outputs{k} ' output'],'Position',[970 y 1000 y+20],'Port',num2str(k));
    add_line(body,[sources{k} '/1'],[outputs{k} ' output/1']);
    add_block('built-in/Outport',[model '/' outputs{k}],'Position',[600 y+40 630 y+60],'Port',num2str(k));
    add_line(model,['Robot/' num2str(k)],[outputs{k} '/1']);
end
for k = 1:2
    inputs = {'Velocity reference','Turn rate reference'}; y = 105+80*(k-1);
    add_block('built-in/Inport',[model '/' inputs{k}],'Position',[70 y 100 y+20],'Port',num2str(k));
    add_line(model,[inputs{k} '/1'],['Robot/' num2str(k)]);
end
add_block('built-in/Mux',[model '/Robot outputs'],'Position',[720 80 725 250],'Inputs','4');
add_block('simulink/Sinks/Scope',[model '/Scope'],'Position',[820 90 870 140]);
add_block('simulink/Sinks/To Workspace',[model '/Log'],'Position',[810 200 950 240], ...
    'VariableName','robotOutputs','SaveFormat','Timeseries','MaxDataPoints','inf');
for k = 1:4
    add_line(model,['Robot/' num2str(k)],['Robot outputs/' num2str(k)]);
end
add_line(model,'Robot outputs/1','Scope/1');
add_line(model,'Robot outputs/1','Log/1');
Simulink.BlockDiagram.arrangeSystem(body);
Simulink.BlockDiagram.arrangeSystem(model);
save_system(model,fullfile(folder,[model '.slx']));
end

function closeSource(source,wasLoaded)
if ~wasLoaded, close_system(source,0); end
end
