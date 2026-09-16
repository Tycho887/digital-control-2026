%% Motor values from one repeatable 6 V step
% The selected pulse starts at 0.020 s and ends at 0.500 s.

logPath = fullfile(fileparts(mfilename('fullpath')), 'src', 'logs', ...
    'device-monitor-260909-113357.log');
stepStart_s = 0.020;
stepEnd_s = 0.500;
window_s = 0.012;

% Motor-side inertia: motor core plus half the robot mass reflected through
% the wheel and gearbox.
robotMass_kg = 0.927;
wheelRadius_m = 0.03;
gearRatio = 9.68;
motorCoreInertia_kgm2 = 1.01e-6;

data = readmatrix(logPath, 'FileType', 'text', 'CommentStyle', '%');
data = data(all(isfinite(data), 2), :);
assert(size(data, 2) >= 16, 'Expected 16 numeric log columns.');

time_s = data(:, 1);
voltage_V = data(:, 4:5);
velocity_rad_s = data(:, 8:9);
current_A = data(:, 15:16);

baseline = time_s >= stepStart_s - window_s & time_s < stepStart_s;
step = time_s >= stepStart_s & time_s <= stepEnd_s;
steady = time_s >= stepEnd_s - window_s & time_s <= stepEnd_s;
assert(any(baseline) && any(step) && any(steady), 'Selected step is outside the log.');

baselineVoltage_V = mean(voltage_V(baseline, :));
baselineCurrent_A = mean(current_A(baseline, :));
voltageStep_V = mean(voltage_V(step, :)) - baselineVoltage_V;
currentResponse_A = sign(voltageStep_V) .* ...
    (current_A(step, :) - baselineCurrent_A);
[peakCurrent_A, peakIndex] = max(currentResponse_A);

% tau is when the current reaches 63.2% of its peak response.
stepTime_s = time_s(step);
tau_s = zeros(1, 2);
for motor = 1:2
    threshold_A = 0.632 * peakCurrent_A(motor);
    crossing = find(currentResponse_A(1:peakIndex(motor), motor) >= threshold_A, 1);
    assert(~isempty(crossing), 'Could not find the 63.2%% current crossing.');

    crossingTime_s = stepTime_s(crossing);
    if crossing > 1
        previous_A = currentResponse_A(crossing - 1, motor);
        current_A_at_crossing = currentResponse_A(crossing, motor);
        fraction = (threshold_A - previous_A) / ...
            (current_A_at_crossing - previous_A);
        crossingTime_s = stepTime_s(crossing - 1) + fraction * ...
            (stepTime_s(crossing) - stepTime_s(crossing - 1));
    end
    tau_s(motor) = crossingTime_s - stepStart_s;
end

V = mean(voltage_V(steady, :));
A = mean(current_A(steady, :));
omega = mean(velocity_rad_s(steady, :));
R = abs(voltageStep_V) ./ peakCurrent_A;
L = R .* tau_s;
Kb = abs((V - R .* A) ./ omega);
D = abs((A .* Kb) ./ omega);
J = motorCoreInertia_kgm2 + (robotMass_kg / 2) * ...
    (wheelRadius_m / gearRatio)^2;

left = struct('V', V(1), 'A', A(1), 'omega', omega(1), ...
    'R', R(1), 'L', L(1), 'Kb', Kb(1), 'D', D(1), 'J', J);
right = struct('V', V(2), 'A', A(2), 'omega', omega(2), ...
    'R', R(2), 'L', L(2), 'Kb', Kb(2), 'D', D(2), 'J', J);

disp('Left motor values:');
disp(left);
disp('Right motor values:');
disp(right);
