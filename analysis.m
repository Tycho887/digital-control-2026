dd = load('device-monitor-250830-081942.log'); % use the filename and directory of your logfile.
h = figure(1000);
hold off
plot(dd(:,1), dd(:,8))
hold on
plot(dd(:,1), dd(:,9))
plot(dd(:,1), dd(:,15))  % column with (most) vertical gyro data
grid on
legend('Velocity left (rad/s)', 'Velocity right (rad/s)', 'Turnrate (??)')
xlabel('Time (sec)')
