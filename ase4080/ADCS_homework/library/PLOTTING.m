close all;
% Plot
MTQ_cmd             = out.MTQ_cmd';
MTQ_mag_accumulated = out.MTQ_cmd_accumulated; 
MTQ_trq             = out.MTQ_trq';
w                   = out.w';
w_mag               = out.w_mag';
TIME                = out.tout / period;

% 시간에 따른 MTQ 출력 값
figure;
plot(TIME, MTQ_cmd(1,:))
hold on
plot(TIME, MTQ_cmd(2,:))
plot(TIME, MTQ_cmd(3,:))
grid on
xlabel('time [orb]')
ylabel('Dipole moment [Am^2]')
legend('X [Am^2]','Y [Am^2]','Z [Am^2]')
ylim([-Max_dipole , Max_dipole]);
title(sprintf('Magnetorquer dipole moment output & K = %2.0f', K_bdot))

% 누적 MTQ 출력 값
figure;
plot(TIME, MTQ_mag_accumulated);
grid on
xlabel('time [orb]')
ylabel('Accumlated dipole moment output [A s m^2]')
title(sprintf('Cumulative magnetorquer dipole moment output & K = %2.0f', K_bdot))

% 실제 MTQ 출력에 의해 만들어진 토크
figure;
plot(TIME, MTQ_trq(1,:))
hold on
plot(TIME, MTQ_trq(2,:))
plot(TIME, MTQ_trq(3,:))
grid on
xlabel('time [orb]')
ylabel('Torque [Nm]')
legend('X [Nm]','Y [Nm]','Z [Nm]')
title(sprintf('Generated Torque & K = %2.0f', K_bdot))

% 위성의 각속도
figure;
plot(TIME, w(1,:))
hold on
plot(TIME, w(2,:))
plot(TIME, w(3,:))
grid on
xlabel('time [orb]')
ylabel('Rotation velocity [deg/s]')
legend('X [deg/s]','Y [deg/s]','Z [deg/s]')
title(sprintf('Rotation velocity of satellite & K = %2.0f', K_bdot))

% 위성의 각속도 크기
figure;
plot(TIME, w_mag)
hold on
grid on
xlabel('time [orb]')
ylabel('Rotation velocity [deg/s]')
title(sprintf('Rotation velocity of satellite & K = %2.0f', K_bdot))