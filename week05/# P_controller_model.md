# P_controller_model.md

## 1. Simulink Model Architecture

The implementation of a digital P-controller requires modifying the continuous-time model to incorporate discrete sampling effects. The Simulink block diagram must be structured with the following sequence:

* **Reference Input:** A Step block configured to output 0.1 m/s, stepping at 0.2 seconds.


* **Input Kinematics:** A gain block labeled `gear/wrad` converting the reference linear velocity to rotational velocity prior to the error calculation.


* **Proportional Controller:** A summing junction to calculate the error signal, connected to a proportional gain block ($K_p$), initially set to 0.05.


* **Saturation:** A voltage limiter restricting the controller output to $\pm 9\text{V}$.


* **Digital-to-Analog (D/A):** A Zero-Order Hold (ZOH) block placed after the controller, utilizing a workspace variable `Ts` to define the sample time.


* **Plant:** The continuous motor model processing the sampled voltage to determine angular velocity.


* **Feedback Loop:** A `detect_delay` block simulating sensor latency, followed by an Analog-to-Digital (A/D) ZOH block (also using `Ts`) that feeds the delayed, sampled velocity back to the initial summing junction.


* **Output Kinematics:** Gain blocks (`1/gear` and `wrad`) transforming the continuous motor angular velocity back into linear velocity (m/s) for data logging to the workspace.



## 2. MATLAB Initialization and Execution Script

The script below initializes the required workspace variables, executes the Simulink model, and generates the necessary plots. The physical parameters are derived from the baseline motor specifications and the basebot setup.

```matlab
%% Motor and Robot Parameters
L = 1e-3;               % Inductance (Henry)
R = 1;                  % Resistance (Ohm)
Kt = 0.01;              % Torque constant (Nm/A)
Kb = Kt;                % Back-EMF constant (V/(rad/s))
J = 1e-5;               % Inertia (Kg m^2) 
D = 1e-5;               % Damping (Nm/(rad/s))
gear = 9.6;             % Gear ratio 9.6:1
wheelRadius = 0.03;     % Wheel radius (m)

%% Sampling Time Configuration
Ts = 0.001;             % Sample time (seconds)
Kp = 0.050;             % Proportional gain

%% Execute Simulation
model = 'basebot_model_sampling';
aaa = sim(model, 0.6);  % Simulate for 0.6 seconds

%% Plot Results
h = figure(171);
plot(aaa.velP.Time, aaa.velP.Data(:,1), 'LineWidth', 2);
hold on;
plot(aaa.velP.Time, aaa.velP.Data(:,2), 'LineWidth', 1.5);
plot(aaa.velP.Time, aaa.velP.Data(:,3), 'LineWidth', 1.5);
legend('ref (m/s)', 'Motor voltage / 10 (V)', 'Velocity (m/s)', 'Location', 'best');
title(sprintf('Sampling: Ts=%.3f sec, Kp=%.3f', Ts, Kp));
xlabel('Time (sec)');
ylabel('Velocity (m/s)');
grid on;
saveas(h, 'sampling_time_1ms_step.png');

```

## 3. Experimental Procedure

To evaluate the impact of digital sampling on system stability, the following procedure is outlined for the simulation environment:

* Execute the model with the baseline sampling time of $T_s = 0.001$ seconds. A stable response with a stationary steady-state error is expected at this baseline.


* Iteratively increase the value of the `Ts` variable in the MATLAB script to simulate longer sampling times.


* Observe the plotted system response after each iteration until the velocity output becomes unstable.


* Identify and record the longest possible sample time that still yields a functional and decent response.