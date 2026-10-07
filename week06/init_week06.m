% Parameters from week05. Run this script before simulating week06.
% Kp acts on motor speed error, in V/(rad/s); references are in m/s.
wheel = struct('L',1e-3,'R',1,'Kt',0.01,'Kb',0.01,'J',1e-5, ...
    'D',1e-5,'gear',9.68,'wheelRadius',0.03,'Kp',0.05,'Ts',0.001, ...
    'voltageLimit',9);
referenceSpeed = 0.1;
stepTime = 0.2;
model = 'week06_wheel_model';
load_system(fullfile(fileparts(mfilename('fullpath')), [model '.slx']));
workspace = get_param(model,'ModelWorkspace');
assignin(workspace,'wheel',wheel);
assignin(workspace,'referenceSpeed',referenceSpeed);
assignin(workspace,'stepTime',stepTime);
