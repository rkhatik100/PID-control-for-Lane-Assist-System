num = conv([1, 10], [34.41, -10.52, 2419]);  % numerator coefficients
den = conv([1, 0, 0], [1, 2.809, 5.295]);    % denominator coefficients

G = tf(num, den);

desired_overshoot = 0.10;  % 10%
desired_settling_time = 2; % seconds
sampling_time = 1/50;      % 50 Hz

[z, p, k] = damp(G);  % Get the poles and zeros of the system
wn = 4/(desired_settling_time * abs(real(max(p))));  % Calculate the desired natural frequency
damping_ratio = (-log(desired_overshoot)) / (sqrt(pi^2 + log(desired_overshoot)^2));  % Calculate the desired damping ratio
desired_poles = roots([1, 2*damping_ratio*wn, wn^2]);  % Calculate the desired poles
Kp = 2*damping_ratio*wn;  % Proportional gain
Ki = wn^2;               % Integral gain
Kd = damping_ratio*2*wn;  % Derivative gain

C = pid(Kp, Ki, Kd);
sys_with_pid = feedback(G*C, 1)

sysd_with_pid = c2d(sys_with_pid, sampling_time, 'zoh')  % Discretize using zero-order hold method
t = 0:0.02:10;              % time vector
ref_signal = sin(t);        % example reference signal
[y, ~, ~] = lsim(sysd_with_pid, ref_signal, t);
[y1,~,~] = lsim(G,ref_signal,t)
plot(y1,ref_s)
plot(t, ref_signal, 'r-', 'LineWidth', 1.5);
hold on;
plot(t, y, 'b--', 'LineWidth', 1.5);
xlabel('time(s)');
ylabel('position signal');
title('System Response with PID Control');
legend('Ideal Position signal', 'position tracking');
grid on;
