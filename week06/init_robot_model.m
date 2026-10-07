% Run from MATLAB to install editable defaults in the robot model workspace.
% Edit the structures here, or call configure_robot_model(wheels,robot) later.
addpath(fileparts(mfilename('fullpath')));
[wheels, robot] = robot_model_defaults();
model = configure_robot_model(wheels,robot);
