function verify_robot_position()
% Verify configured offsets and frame orientation with the actual wheel model.
folder = fileparts(mfilename('fullpath'));
[wheels,robot] = robot_model_defaults();
robot.stopTime = 1;
model = configure_robot_model(wheels,robot);
commands = [0 0.1 0.5; 1 0.1 0.5];
inputs = Simulink.SimulationData.Dataset;
for k = 1:2
    signal = timeseries(commands(:,k+1),commands(:,1));
    inputs = inputs.addElement(setinterpmethod(signal,'zoh'));
end
input = Simulink.SimulationInput(model).setExternalInput(inputs);
a = sim(input);
robot.initialX = 1.2; robot.initialY = -0.4;
robot.initialAngle = pi/2; robot.initialDistance = 0.7;
configure_robot_model(wheels,robot);
b = sim(input);
time = linspace(0,1,1001)';
first = interp1(a.robotOutputs.Time,a.robotOutputs.Data,time);
second = interp1(b.robotOutputs.Time,b.robotOutputs.Data,time);
expected = first+[robot.initialDistance robot.initialAngle robot.initialX robot.initialY];
assert(max(abs(second-expected),[],'all')<1e-5, ...
    'Initial offsets or initial-frame projection are incorrect.');
assert(first(end,3)>0 && first(end,4)>0,'Positive arc must move forward and left.');
% Analytical quarter-circle check of the numerical projection equations.
t = linspace(0,pi/2,10001)'; v = 0.1; w = 1;
x = cumtrapz(t,v*cos(w*t)); y = cumtrapz(t,v*sin(w*t));
assert(abs(x(end)-v/w)<1e-8 && abs(y(end)-v/w)<1e-8);
configure_robot_model(wheels,robot_model_defaults_robot());
fprintf('PASS: initial offsets, fixed initial frame, positive arc and projection equations.\n');
end

function robot = robot_model_defaults_robot()
[~,robot] = robot_model_defaults();
end
