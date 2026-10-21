%% motor model parameters
close all
clear;
%% Motor parameters
L = 1e-3; % Henry
R = 1; % Ohm
Kt = 0.01; % Nm/A
Kb = Kt;   % V/(rad/s)
J = 1e-5;  % (Kg m^2) = Nm/(rad/s^2)
D = 1e-5;  % Nm/(rad/s)
gear = 9.68;
wheelRadius = 0.03; % (m)
r2m = wheelRadius/gear; % (radian/s to meter/s)

%% simulate (Steps)
model ='motor_model_23b';