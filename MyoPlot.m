hFig = figure(1);
%set(hFig, 'Position', [20 20 800 680])

AcceData = [Acce; AcceData(1:end-1,:)];
GyroData = [Gyro; GyroData(1:end-1,:)];
OriData = [Ori; OriData(1:end-1,:)];

subplot(2,2,1),
plot(AcceData);
axis([0 30 -4 4]);
grid on;
title('Accelerometer');
xlabel('Iterations');
ylabel('[g]');
%legend('x','y','z')

subplot(2,2,3),
plot(GyroData);
axis([0 30 -2*pi 2*pi]);
grid on;
title('Gyroscope');
xlabel('Iterations');
ylabel('[rad/s]');
%legend('wx','wy','wz')

subplot(2,2,2),
plot(OriData);
axis([0 30 -pi pi]);
grid on;
title('Rotation');
xlabel('Iterations');
ylabel('[rad]');
%legend('pitch','roll','yaw')

subplot(2,2,4),
bar(EMG,'r');
axis([0 8 -128 128]);
grid on;
title('EMG strength');
xlabel('Sensor number');