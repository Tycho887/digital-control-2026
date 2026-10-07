%% motor model parameters
close all
clear;
%% Motor parameters
L = 0.00380;
R = 2.84;
Kt = 0.007956;
Kb = Kt;
J = 5.4619e-6;
D = 3.323e-6;
gear = 9.68;
wrad = 0.03; % (m)
r2m = wrad/gear; % (radian/s to meter/s)
Kp = 0.05;
B = 0.15;

