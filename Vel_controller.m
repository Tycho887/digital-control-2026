s = tf('s');

% Phase margin = 70 degrees
% Ni = 5
% Based on right wheel

L = 0.00380;
R = 2.84;
Kt = 0.007956;
Kb = Kt;
J = 5.4619e-6;
D = 3.323e-6;
Gs = Kt / ((L*s + R)*(J*s + D) + Kt*Kb);

Kp =  1 /(10^(17.3/20));
tau_i = 0.0714;
Cs = Kp * (tau_i*s + 1) / (tau_i*s);


Gol = Cs * Gs; 
Gcl = feedback(Gol,1);




figure;
step(Gcl);
grid on;
title('Closed-Loop Step Response');
disp(Gcl)
