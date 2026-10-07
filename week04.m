% Define the transfer function
s = tf('s');
Gs = 1 / (1 + s);
Gz = c2d(Gs, 0.1, '');
step(Gs,Gz)
disp(Gz)