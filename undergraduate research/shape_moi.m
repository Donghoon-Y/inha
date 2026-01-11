clc; clear; close all;
addpath('function')


T = [0; 0; 0];

J_disk = diag([56.0, 56.0, 64.0]);
J_box  = diag([27.3, 48.2, 54.2]);

q0  = [0; 0; 0; 1];
d2r = pi/180;
w0  = [0.01*d2r; 0.02*d2r; 10*d2r]; % rad/s

x0 = [q0; w0];

dt     = 0.01;
t_span = [0, 1000];

% --- Simulate ---
[t_disk, y_disk] = rungekutta4(@(t, x) attitude_ode(t, x, J_disk, T), t_span, x0, dt);
[t_box,  y_box ] = rungekutta4(@(t, x) attitude_ode(t, x, J_box,  T), t_span, x0, dt);

% --- Split states ---
q_disk = y_disk(1:4, :);
w_disk = y_disk(5:7, :);

q_box  = y_box(1:4, :);
w_box  = y_box(5:7, :);

% --- Angular momentum histories ---
N_disk = size(w_disk, 2);
N_box  = size(w_box,  2);

h_disk = zeros(3, N_disk);
h_box  = zeros(3, N_box);

for k = 1:N_disk
    h_disk(:,k) = J_disk * w_disk(:,k);
end
for k = 1:N_box
    h_box(:,k) = J_box * w_box(:,k);
end

% --- Energy & |h| ---
E_disk    = 0.5 * sum(w_disk .* (J_disk*w_disk), 1);
E_box     = 0.5 * sum(w_box  .* (J_box *w_box ), 1);
hmag_disk = vecnorm(h_disk, 2, 1);
hmag_box  = vecnorm(h_box , 2, 1);

h_disk_i = zeros(3, N_disk);
h_box_i = zeros(3, N_box);


for k = 1:N_disk
    C_bi = dcm_q(q_disk(:,k)); 
    h_disk_i(:,k) = C_bi.' * h_disk(:,k);     
end

for k = 1:N_box
    C_bi = dcm_q(q_box(:,k)); 
    h_box_i(:,k) = C_bi.' * h_box(:,k);     
end


% --- Euler angles from quaternion (3-2-1: yaw-pitch-roll) ---
eul_box = zeros(3, N_box);  

for k = 1:N_box
    C_bi = dcm_q(q_box(:,k));          
    eul_box(:,k) = dcm2eul321(C_bi);   
end

phi   = eul_box(1,:);   % roll
theta = eul_box(2,:);   % pitch
psi   = eul_box(3,:);   % yaw


% Quaternion comparison
figure;
for i = 1:4
    subplot(4,1,i); hold on;
    plot(t_disk, q_disk(i,:), 'r', 'LineWidth', 1, 'DisplayName', 'disk');
    plot(t_box,  q_box(i,:),  'b', 'LineWidth', 1, 'DisplayName', 'box', LineStyle='--');
    grid on;
    ylabel(sprintf('q_%d', i));
    if i==4, xlabel('Time [s]'); end
    if i==1, legend(); end
    hold off;
end
sgtitle('Quaternion Time History (disk vs box)');

% Angular velocity comparison
figure;
for i = 1:3
    subplot(3,1,i); hold on;
    plot(t_disk, w_disk(i,:), 'r', 'LineWidth', 1, 'DisplayName', 'disk');
    plot(t_box,  w_box(i,:),  'b', 'LineWidth', 1, 'DisplayName', 'box');
    grid on;
    ylabel(sprintf('\\omega_%d [rad/s]', i));
    if i==3, xlabel('Time [s]'); end
    if i==1, legend(); end
    hold off;
end
sgtitle('Angular Velocity (disk vs box)');

% Angular momentum comparison
figure;
for i = 1:3
    subplot(3,1,i); hold on;
    plot(t_disk, h_disk(i,:), 'r', 'LineWidth', 1, 'DisplayName', 'disk');
    % plot(t_box,  h_box(i,:),  'b', 'LineWidth', 1, 'DisplayName', 'box', LineStyle='--');
    grid on;
    ylabel(sprintf('h_%d', i));
    if i==3, xlabel('Time [s]'); end
    if i==1, legend(); end
    hold off;
end
sgtitle('Angular Momentum Components (disk vs box)');

% Energy comparison
figure; hold on;
plot(t_disk, E_disk, 'LineWidth', 1, 'DisplayName', 'disk');
plot(t_box,  E_box,  'LineWidth', 1, 'DisplayName', 'box');
grid on; title('Rotational Energy'); xlabel('Time [s]'); ylabel('E');
legend(); hold off;

% |h| comparison
figure; hold on;
plot(t_disk, hmag_disk, 'LineWidth', 1, 'DisplayName', 'disk');
plot(t_box,  hmag_box,  'LineWidth', 1, 'DisplayName', 'box');
grid on; title('|h|'); xlabel('Time [s]'); ylabel('|h|');
legend(); hold off;

% Angular momentum comparison
figure;
for i = 1:3
    subplot(3,1,i); hold on;
    plot(t_disk, h_disk_i(i,:), 'r', 'LineWidth', 1.2, 'DisplayName', 'disk');
    %plot(t_box,  h_box_i(i,:),  'b', 'LineWidth', 1, 'DisplayName', 'box', LineStyle='--');
    grid on;
    ylabel(sprintf('h_%d', i));
    if i==3, xlabel('Time [s]'); end
    if i==1, legend(); end
    hold off;
end
sgtitle('Angular Momentum Components in ECI (disk vs box)');

%Euler angle 
figure;
subplot(3,1,1); plot(t_box, phi*180/pi);   grid on; ylabel('\phi (roll) [deg]');
subplot(3,1,2); plot(t_box, theta*180/pi); grid on; ylabel('\theta (pitch) [deg]');
subplot(3,1,3); plot(t_box, psi*180/pi);   grid on; ylabel('\psi (yaw) [deg]'); xlabel('Time [s]');
sgtitle('Euler angles (3-2-1) from quaternion');