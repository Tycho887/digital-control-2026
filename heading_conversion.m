Ts = 0.001; 

num_s = Kph * [head_tau_d, 1];
den_s = [head_alpha * head_tau_d, 1];
sys_c = tf(num_s, den_s);

sys_d = c2d(sys_c, Ts, 'tustin');
[num_d, den_d] = tfdata(sys_d, 'v');

a0 = den_d(1);
num_d = num_d / a0;
den_d = den_d / a0;

b0 = num_d(1);
b1 = num_d(2);
b2 = 0.0; 

a1 = den_d(2);
a2 = 0.0; 

disp([Kph, head_tau_d, head_alpha, Ts]);
disp([b0, b1, b2, a1, a2]);

