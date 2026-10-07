function model = configure_robot_model(wheels, robot)
% Install parameters in the model workspace; changes persist if model is saved.
folder = fileparts(mfilename('fullpath'));
model = 'week06_robot_model';
assert(numel(wheels)==2,'Supply wheels(1) for left and wheels(2) for right.');
positive = {'L','R','Kt','Kb','J','D','gear','wheelRadius','Kp','Ts','voltageLimit'};
for k = 1:2
    for j = 1:numel(positive)
        value = wheels(k).(positive{j});
        assert(isnumeric(value)&&isscalar(value)&&isfinite(value)&&value>0, ...
            'Wheel %d field %s must be finite and positive.',k,positive{j});
    end
    for name = {'initialMotorSpeed','initialCurrent','initialSensorSpeed','diagnosticVoltageScale'}
        value = wheels(k).(name{1});
        assert(isnumeric(value)&&isscalar(value)&&isfinite(value), ...
            'Wheel %d field %s must be a finite scalar.',k,name{1});
    end
end
for name = {'trackWidth','stopTime','relativeTolerance','absoluteTolerance','maxStep'}
    value = robot.(name{1});
    assert(isnumeric(value)&&isscalar(value)&&isfinite(value)&&value>0, ...
        'robot.%s must be finite and positive.',name{1});
end
for name = {'initialDistance','initialAngle'}
    value = robot.(name{1});
    assert(isnumeric(value)&&isscalar(value)&&isfinite(value), ...
        'robot.%s must be a finite scalar.',name{1});
end
assert(robot.maxStep<=min([wheels.Ts])/10, ...
    'Set robot.maxStep to at most min([wheels.Ts])/10.');
load_system(fullfile(folder,[model '.slx']));
workspace = get_param(model,'ModelWorkspace');
assignin(workspace,'wheels',wheels);
assignin(workspace,'robot',robot);
set_param(model,'StopTime','robot.stopTime','RelTol','robot.relativeTolerance', ...
    'AbsTol','robot.absoluteTolerance','MaxStep','robot.maxStep');
end
