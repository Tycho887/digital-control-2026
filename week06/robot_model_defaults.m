function [wheels, robot] = robot_model_defaults()
% MATLAB-owned defaults for the robot and the previously tuned wheel loops.
folder = fileparts(mfilename('fullpath'));
tuned = load(fullfile(folder,'basebot_p_gains.mat'),'Kp','motorParameters');
wheels = tuned.motorParameters;
for k = 1:2
    wheels(k).Kp = tuned.Kp(k);
    wheels(k).initialMotorSpeed = 0; % rad/s, motor shaft
    wheels(k).initialCurrent = 0; % A
    wheels(k).initialSensorSpeed = 0; % rad/s, delayed motor measurement
    wheels(k).diagnosticVoltageScale = 0.1;
end
robot = struct('trackWidth',0.14,'initialDistance',0,'initialAngle',0, ...
    'stopTime',3,'relativeTolerance',1e-6,'absoluteTolerance',1e-9, ...
    'maxStep',min([wheels.Ts])/10);
end
