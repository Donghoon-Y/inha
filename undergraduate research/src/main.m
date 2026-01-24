clc; clear;

%쿼터니언은 JPL Notation 사용

thisFile = mfilename('fullpath');
thisDir  = fileparts(thisFile);
addpath(genpath(thisDir)); 

% Parameter Setting
mu = 398600;        % [km3/s2] Earth gravitational constant
R = 6371;           % [km] Earth radius
a = 600+R;          % [km] semi-major axis
e = 0;              % [-] eccentricity
inc = 97.8;         % [deg] inclination
raan = 0;           % [deg] 승교점이각
aop = 0;            % [deg] 근점이각
lat = 37.2302;      % [deg] 위도 (항공우주산학융합원)
lon = 126.3925;     % [deg] 경도
alt = 0;            % [m] 해발고도
angle = 10;         % [deg] 최소교신각도 (Elevation Mask)

% Earth sphere for plotting
[xe, ye, ze] = sphere(80);  
Re = R;                      
xe = Re * xe; ye = Re * ye; ze = Re * ze;

% Initial Position/Velocity (PQW)
r_pqw = [a; 0; 0];                   
v_pqw = [0; sqrt(mu/a); 0];         

% Ground Station (ECEF) [km]
x_g_ecef = R*[cos(deg2rad(lat))*cos(deg2rad(lon));
              cos(deg2rad(lat))*sin(deg2rad(lon));
              sin(deg2rad(lat))];

% PQW to ECI rotation
R_W = [cos(deg2rad(raan)) -sin(deg2rad(raan)) 0;
       sin(deg2rad(raan)) cos(deg2rad(raan)) 0;
       0 0 1];
R_i = [1 0 0;
       0 cos(deg2rad(inc)) -sin(deg2rad(inc));
       0 sin(deg2rad(inc)) cos(deg2rad(inc))];
R_w = [cos(deg2rad(aop)) -sin(deg2rad(aop)) 0;
       sin(deg2rad(aop)) cos(deg2rad(aop)) 0;
       0 0 1];
pqw2eci = R_W*R_i*R_w;

% Time Settings
t_kst = datetime(2025,12,19,0,0,0,'TimeZone','Asia/Seoul');
t_utc0 = t_kst;
t_utc0.TimeZone = 'UTC';


% Orbit Simulation (Two Body)
r0 = pqw2eci*r_pqw;
v0 = pqw2eci*v_pqw;
T_period = 2*pi*sqrt(a^3/mu);
x0 = [r0; v0];
tspan = [0 8*T_period]; 
dt = 0.1;

fprintf("Orbit Simulation Start.......\n");
[t, x] = rungekutta4(@(t,x) odeTwoBody(t,x,mu), tspan, x0, dt);

% Orbit results
r_eci = x(1:3, :).'; % [km] N x 3
v_eci = x(4:6, :).'; % [km/s]

% ECI to ECEF Conversion (for Ground Track)
r_ecef = zeros(size(r_eci));
v_ecef = zeros(size(v_eci));
t_utc = t_utc0 + seconds(t);
cnt = 0;

fprintf("Converting ECI to ECEF (Original).......\n");
for i = 1:length(t)
    utc_vec = datevec(t_utc(i));
    mjd = mjuliandate(utc_vec);
    %pm = polarMotion(mjd);
    [re_m, ve_m] = eci2ecef(utc_vec, r_eci(i, :)*1000, v_eci(i, :)*1000);
    
    r_ecef(i,:) = re_m/1000;
    v_ecef(i,:) = ve_m/1000;
   
    if mod(i, 500) == 0  % 출력 빈도 조절
        fprintf('%d ', i);
        cnt = cnt +1 ;
        if mod(cnt, 10) == 0
            fprintf('\n');   
        end
    end
end
fprintf('\n');
disp("Orbit Calculation Complete.");

%%
% Sun Position Calculation (N x 3)
[r_sun_eci_approx, u_sun_eci_approx] = sun_eci_from_utc(t_utc);
rho_approx = r_sun_eci_approx - r_eci;
rho_approx_hat = rho_approx ./ vecnorm(rho_approx, 2, 2);

% Rigid Body Parameters
J = diag([0.070, 0.071, 0.007]);
q0 = [0.2; -0.1; 0.3; 0.92];  % Scalar Last, 디텀블링 고려
q0 = q0/norm(q0);
w0 = deg2rad([1; -2; 0.5]);   % rad/s
x0_att = [q0; w0];

% Controller Gains
Ts = 30; % 2%오차에 도달하는 시간 (10)
zeta = 1/sqrt(2); %오버슛 적게 하기 위해서 1/sqrt(2)
wn = 4/(Ts*zeta);
Kp = diag(diag(J) * wn^2);
Kd = diag(2*zeta*diag(J)*wn);  
tau_max = [0.1 * 1e-3; 0.1 * 1e-3; 0.1 * 1e-3];    
%tau_max = [inf;inf;inf];

% Simulate (Switching ODE 호출)
fprintf("Attitude Simulation Start (Switching Mode).......\n");

[t_att, x_att] = q_rungekutta4(@(tt,xx) attitude_mode_switch(tt, xx, J, Kp, Kd, tau_max, r_eci, v_eci, rho_approx_hat, x_g_ecef*1000, angle, dt), tspan, x0_att, dt);

q_hist = x_att(1:4, :).'; 
w_hist = x_att(5:7, :).';

t_att_utc = t_utc0 + seconds(t_att);

N = length(t_att);
pointing_err_deg = zeros(N,1);
control_torque = zeros(N,3);
mode_hist = zeros(N,1); % 0: Sun, 1: Ground

x_b = [1; 0; 0]; % Body Z-axis (Antenna)
We = 7.292115e-5; %지구자전 각속도

fprintf("Analyzing Results.......\n");
for i = 1:N 
    % Current State
    t_now = t_att(i);
    q_now = q_hist(i, :).'; 
    w_now = w_hist(i, :).';
    
    k = i;
    if k > size(r_eci, 1), k = size(r_eci, 1); end
    
    % Geometry Reconstruction (Simulate what the ODE did)
    r_sat_m = r_eci(k, :).' * 1000;
    
    % Ground Station Position in ECI
    theta = We * t_now;
    R_ecef2eci = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
    r_gs_m = R_ecef2eci * (x_g_ecef * 1000);
    
    % Visibility Check
    r_rel_m = r_gs_m - r_sat_m;
    u_zenith = r_gs_m / norm(r_gs_m);
    u_gs2sat = -r_rel_m / norm(r_rel_m);
    elv_deg = rad2deg(asin(dot(u_gs2sat, u_zenith)));
    
    % Mode Determination & Target Vector
    if elv_deg >= angle
        % [Mode: GROUND Station]
        mode_hist(i) = 1; 
        target_vec_eci = r_rel_m / norm(r_rel_m); 
        % Control Torque Calculation (Approximate for plotting)
        % 시각화용으로 간단히 에러에 비례한다고 가정하거나 0으로 둠
    else
        % [Mode: Sun]
        mode_hist(i) = 0;
        target_vec_eci = rho_approx_hat(k, :).'; % Look at Sun
    end
    
    % Pointing Error Calculation
    % (Target Vector를 Body Frame으로 변환하여 Z축과 비교)
    C_bi = dcm_q(q_now); 
    target_vec_body = C_bi * target_vec_eci;
    target_vec_body = target_vec_body / norm(target_vec_body);
    
    dot_val = dot(target_vec_body, x_b);
    dot_val = max(min(dot_val, 1), -1);
    pointing_err_deg(i) = acosd(dot_val);
end

%%Plot
% (1) 3D Orbit
figure('Name','3D Orbit','Color','w'); hold on;
surf(xe, ye, ze, 'FaceColor',[0.6 0.8 1], 'EdgeColor','none', 'FaceAlpha',0.3, 'DisplayName','Earth');
plot3(r_eci(:,1), r_eci(:,2), r_eci(:,3),'-b', 'LineWidth',1, 'DisplayName','Orbit');
% 지상국 위치 표시 
plot3(x_g_ecef(1), x_g_ecef(2), x_g_ecef(3), 'rp', 'MarkerSize',10, 'MarkerFaceColor','r', 'DisplayName','GS (t=0)');
axis equal; grid on; xlabel('X [km]'); ylabel('Y [km]'); zlabel('Z [km]');
legend; view(3); title('Satellite Trajectory (ECI)');

% (2) Ground Track
r_ecef_m = r_ecef*1e3;
lla = ecef2lla(r_ecef_m);
s_lat = lla(:,1); s_lon = lla(:,2);
s_lon = wrapTo180(s_lon);
figure('Name','Ground Track','Color','w'); hold on;
plot(s_lon, s_lat, '.', 'MarkerSize', 2, 'Color',"b", 'DisplayName',"Trajectory");
scatter(lon, lat, 50, 'r', 'filled', 'DisplayName',"Incheon GS");
grid on; xlabel('Longitude [deg]'); ylabel('Latitude [deg]');
title('Ground Track'); legend; xlim([-180 180]); ylim([-90 90]);

% (3) Mode & Pointing Error 
figure('Name', 'Control Performance', 'Color', 'w');

% Subplot 1: Mode History
subplot(3,1,1);
area(t_att_utc, mode_hist, 'FaceColor', [0.2 0.8 0.2], 'FaceAlpha', 0.3);
ylim([-0.1 1.1]);
yticks([0 1]); yticklabels({'Sun', 'Ground'});
grid on; ylabel('Mode');
title('Attitude Control Mode Switching');

% Subplot 2: Pointing Error
subplot(3,1,2);
plot(t_att_utc, pointing_err_deg, 'LineWidth', 1.2, 'Color', 'k');
grid on; ylabel('Error [deg]');
title('Pointing Error (Relative to Active Target)');
yline(0, '--r'); 
ylim([0, max(10, max(pointing_err_deg)*1.1)]); 

% Subplot 3: Angular Velocity
subplot(3,1,3);
plot(t_att_utc, rad2deg(w_hist), 'LineWidth', 1);
grid on; ylabel('Ang Vel [deg/s]'); xlabel('Time [sec]');
legend('\omega_x', '\omega_y', '\omega_z');
title('Body Angular Velocity');

% (4) Quaternion History
figure('Name', 'Quaternion', 'Color', 'w');
plot(t_att_utc, q_hist, 'LineWidth', 1.2);
grid on; xlabel('Time [sec]'); ylabel('Quaternion');
legend('q_x', 'q_y', 'q_z', 'q_w (Scalar)');
title('Quaternion History');

%%
fprintf('\nStarting Real-Time Animation...\n');

play_speed = 50; 
step = play_speed; 

f_anim = figure('Name', 'Real-Time Satellite Monitor', 'Color', 'w', 'Position', [100, 100, 1200, 900]);
axis equal; grid on; hold on;
xlabel('X [km]'); ylabel('Y [km]'); zlabel('Z [km]');
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k', 'FontSize', 12);
view(3);
xlim([-a-3000, a+3000]); ylim([-a-3000, a+3000]); zlim([-a-3000, a+3000]);


surf(xe, ye, ze, 'FaceColor', [0.1, 0.3, 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.6);
plot3(r_eci(:,1), r_eci(:,2), r_eci(:,3), 'Color', [0.4 0.4 0.4], 'LineWidth', 0.5);

h_sat = plot3(0,0,0, 'co', 'MarkerSize', 10, 'MarkerFaceColor', 'c', 'DisplayName', 'Satellite');
h_gs = plot3(0,0,0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Incheon GS');
h_link = plot3([0 0], [0 0], [0 0], 'y-', 'LineWidth', 3, 'DisplayName', 'Comm Link');
h_sun = quiver3(0,0,0, 0,0,0, 0, 'Color', [1 0.6 0], 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName', 'Sun Vector');
h_z_axis = quiver3(0,0,0, 0,0,0, 0, 'Color', 'c', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName', 'Body x-axis');
h_text = title('Initializing...', 'Color', 'k', 'FontSize', 15, 'FontWeight', 'bold');

legend([h_sat, h_gs, h_link, h_sun, h_z_axis], 'Location', 'northeast');


% 4. 애니메이션 루프
N_total = length(t_att);
vec_len = 4000; 

fprintf('Animation Playing... \n');

for k = 1 : step : N_total
    % (1) 현재 인덱스 및 시간
    idx = k;
    if idx > size(r_eci,1), idx = size(r_eci,1); end
    curr_time = t_att(k);
    curr_time_utc = t_att_utc(k);
    
    % (2) 위치 업데이트
    % 위성 (ECI)
    sat_pos = r_eci(idx, :);
    set(h_sat, 'XData', sat_pos(1), 'YData', sat_pos(2), 'ZData', sat_pos(3));
    
    % 지상국 (ECI 회전 반영)
    theta = We * curr_time;
    R_rot = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
    gs_pos = (R_rot * x_g_ecef * 1000 / 1000)'; % [km]
    set(h_gs, 'XData', gs_pos(1), 'YData', gs_pos(2), 'ZData', gs_pos(3));
    
    % (3) 자세 및 벡터 업데이트
    q_curr = q_hist(k, :);
    % 사용자 정의 함수 dcm_q 사용 (Inertial -> Body 가정 시 Transpose 주의)
    C_bi = dcm_q(q_curr); 
    
    % Body Z축을 ECI로 변환 (C_bi' * [0;0;1])
    z_body = [1;0;0];
    z_eci = (C_bi' * z_body)';
    
    % 태양 벡터
    sun_vec = rho_approx_hat(idx, :);
    
    % 화살표 그리기
    set(h_z_axis, 'XData', sat_pos(1), 'YData', sat_pos(2), 'ZData', sat_pos(3), ...
                  'UData', z_eci(1)*vec_len, 'VData', z_eci(2)*vec_len, 'WData', z_eci(3)*vec_len);
              
    set(h_sun, 'XData', sat_pos(1), 'YData', sat_pos(2), 'ZData', sat_pos(3), ...
               'UData', sun_vec(1)*vec_len, 'VData', sun_vec(2)*vec_len, 'WData', sun_vec(3)*vec_len);
    
    % (4) 모드별 시각화 (Ground Tracking vs Sun Pointing)
    time_str = datestr(curr_time_utc, 'yyyy-mm-dd HH:MM:SS');
    if mode_hist(k) == 1 % Ground Mode
        % 링크 연결 (노란선 ON)
        set(h_link, 'XData', [sat_pos(1) gs_pos(1)], ...
                    'YData', [sat_pos(2) gs_pos(2)], ...
                    'ZData', [sat_pos(3) gs_pos(3)], 'Visible', 'on');
        
        % 태양 화살표 숨김 (집중)
        set(h_sun, 'Visible', 'on');
        
        status_msg = sprintf('[%s] UTC | Mode: GROUND TRACKING', time_str);
        set(h_text, 'String', status_msg, 'Color', 'g');
        
    else % Sun Mode
        % 링크 끊김 (노란선 OFF)
        set(h_link, 'Visible', 'off');
        
        % 태양 화살표 보이기
        set(h_sun, 'Visible', 'on');
        
        status_msg = sprintf('[%s] UTC | Mode: SUN POINTING', time_str);
        set(h_text, 'String', status_msg, 'Color', 'k');
    end
    
    drawnow; 
end
fprintf('Animation Finished.\n');

%%
% 1. Ground Mode인 인덱스만 추출
idx_ground = find(mode_hist == 1);

if isempty(idx_ground)
    fprintf('\n[Result] ⚠️ 지상국과 접속한 구간이 없습니다. (Check Angle or Time)\n');
else
    % 2. 해당 구간의 오차값 추출
    ground_errors = pointing_err_deg(idx_ground);
    ground_times = t_att_utc(idx_ground);
    
    % 3. 정착 시간(Settling Time) 이후의 오차만 보기 (Steady State)
    % 모드 전환 직후에는 위성이 회전하느라 오차가 큽니다.
    % 따라서 처음 50초(Ts) 정도는 제외하고 그 뒤에 얼마나 정밀한지 봐야 합니다.
    
    % 간단하게 접속 후반부 50% 데이터만 가지고 정밀도 분석
    cut_idx = round(length(ground_errors) * 0.5); 
    steady_errors = ground_errors(cut_idx:end);
    
    % 4. 통계 출력
    fprintf('\n============================================================\n');
    fprintf('   GROUND TARGETING PERFORMANCE REPORT\n');
    fprintf('============================================================\n');
    fprintf('총 접속 시간:       %.1f 초\n', length(ground_errors) * dt);
    fprintf('------------------------------------------------------------\n');
    fprintf('[전체 접속 구간] (회전 기동 포함)\n');
    fprintf('  - 최대 오차 (Max):  %.4f 도\n', max(ground_errors));
    fprintf('  - 평균 오차 (Mean): %.4f 도\n', mean(ground_errors));
    fprintf('------------------------------------------------------------\n');
    fprintf('[안정화 이후 구간] (Steady State, 후반 50%%)\n');
    fprintf('  - 최대 오차 (Max):  %.6f 도\n', max(steady_errors));
    fprintf('  - 평균 오차 (Mean): %.6f 도\n', mean(steady_errors));
    fprintf('  - RMSE 오차:        %.6f 도\n', rms(steady_errors));
    fprintf('============================================================\n\n');

    % 5. 오차 확대 그래프 (지상국 구간만)
    figure('Name', 'Ground Tracking Error Zoom', 'Color', 'w');
    plot(ground_times, ground_errors, 'r-', 'LineWidth', 1.5);
    grid on; xlabel('Time [UTC]'); ylabel('Pointing Error [deg]');
    title('Ground Tracking Accuracy (Zoomed In)');
    
    % y축 스케일을 오차에 맞춰서 조정 (로그 스케일 아님)
    % 안정화 이후 오차가 작다면 y축을 확대
    if max(steady_errors) < 1
        ylim([0, max(ground_errors)*1.1]); 
    end
end
% =========================================================================