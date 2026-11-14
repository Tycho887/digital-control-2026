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
Kp_vel_left = 0.05;
Kp_vel_right = 0.04;
B = 0.15;
s = tf('s');
Ki_vel_left = 0.1;
Ki_vel_right = 0.1;
Kpt = 5; % Kp Turnrate controller
taui_t = 0.2; % Taui turn controller
KpLT = 25; % Kp for Turn limitor
KpLV = 25; % Kp for Vel limitor
Lim_VU = 0.5; % upper limit vel limiter
Lim_VL = -0.5; 
Lim_TU = 0.7;% upper limit turn limiter
Lim_TL = -0.7;
Kph = 2.5;
uh = 1;
head_tau_d = 0.8;
head_alpha = 0.5;



