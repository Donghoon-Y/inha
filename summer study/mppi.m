clc; clear all;

%저궤도 상황이라고 가정
mu = 398600;
R = 6300;
r = 400;
a = R+r;
n = sqrt(mu/a^3);
x0 = [50; 200; -50; 1; 3; -5]/1000;
xf = [0; 0; 0; 0; 0; 0];

A = [0, 0, 0, 1, 0, 0;
    0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 1;
    3*n^2, 0, 0, 0, 2*n, 0;
    0, 0, 0, -2*n, 0, 0;
    0, 0, -n^2, 0, 0, 0];

B = [0, 0, 0;
    0, 0, 0;
    0, 0, 0;
    1, 0, 0;
    0, 1, 0;
    0, 0, 1];

Q = diag([10, 10, 10, 1, 1, 1]); % 위치 오차에 더 큰 가중치 부여
R = diag([0.1, 0.1, 0.1]);       % 제어 입력 가중치

k = lqr(A, B, Q, R);

A_cal = (A-B*k);

dt = 0.1;
t_final = 20;
tspan = 0:dt:t_final;

[t ,x] = ode45(@(t,x) A_cal*x, tspan, x0);

figure; 
plot3(x(:,1)*1000, x(:,2)*1000, x(:,3)*1000, 'b-');
hold on;
grid on; 
axis equal;
plot3(x0(1)*1000, x0(2)*1000, x0(3)*1000, 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
plot3(xf(1)*1000, xf(2)*1000, xf(3)*1000, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 10);

title('LQR Satellite Trajectory', 'FontSize', 14);
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
legend('LQR Trajectory', 'Start', 'Final', 'Location', 'northeast');
view(45, 25); 

figure;
subplot(3,1,1);
plot(t, x(1:end ,4));
title('Cotrol Inputs','FontSize',13);
legend("Ux");
grid on;

subplot(3,1,2);
plot(t, x(1:end,5));
legend("Uy");
ylabel("Inputs(m/s^2", "FontSize",13)
grid on;

subplot(3,1,3);
plot(t, x(1:end,6));
legend("Uz");
xlabel("Time(s)", FontSize=13)
grid on;

