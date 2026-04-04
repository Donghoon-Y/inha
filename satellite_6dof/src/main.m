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
t_kst = datetime(2023,12,19,0,0,0,'TimeZone','Asia/Seoul');
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
    % pm = polarMotion(mjd);
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
Ts = 40; % 2%오차에 도달하는 시간 (10)
zeta = 1/sqrt(2); %오버슛 적게 하기 위해서 1/sqrt(2)
wn = 4/(Ts*zeta);
Kp = diag(diag(J) * wn^2);
Kd = diag(2*zeta*diag(J)*wn);  
tau_max = [0.1 * 1e-3; 0.1 * 1e-3; 0.1 * 1e-3];    
%tau_max = [inf;inf;inf];

% Simulate (Switching ODE 호출)
fprintf("Attitude Simulation Start (Switching Mode).......\n");

r_gs_ecef_m = x_g_ecef*1000;

point = length(t_utc);
r_gs_eci_m = zeros(point, 3);

for i = 1:point
    r_temp = ecef2eci(t_utc(i), r_gs_ecef_m);
    r_gs_eci_m(i, :) = r_temp;

    if mod(i, 1000) == 0 
        fprintf('Progress: %d / %d (%.1f%%)\n', i, point, (i/point)*100);
    end
end

%%

[t_att, x_att] = q_rungekutta4(@(tt,xx) attitude_mode_switch(tt, xx, J, Kp, Kd, tau_max, r_eci, v_eci, rho_approx_hat, r_gs_ecef_m, angle, dt, t_utc0), tspan, x0_att, dt);

q_hist = x_att(1:4, :).'; 
w_hist = x_att(5:7, :).';

t_att_utc = t_utc0 + seconds(t_att);

N = length(t_att);
pointing_err_deg = zeros(N,1);
control_torque = zeros(N,3);
mode_hist = zeros(N,1); % 0: Sun, 1: Ground

x_b = [1; 0; 0]; % Body Z-axis (Antenna)

fprintf("Analyzing Results.......\n");
for i = 1:N
    % 현재 시점의 ECI 좌표들
    r_sat_eci_m = r_eci(i, :).' * 1000;
    r_gs_now_eci_m = r_gs_eci_m(i, :).';
    
    % 지상국 상대 벡터 및 고도각(Elevation) 계산
    r_rel_eci_m = r_gs_now_eci_m - r_sat_eci_m;
    u_zenith = r_gs_now_eci_m / norm(r_gs_now_eci_m);
    u_gs2sat = -r_rel_eci_m / norm(r_rel_eci_m);
    elv_deg = rad2deg(asin(dot(u_gs2sat, u_zenith)));
    
    % 모드 판별 및 타겟 벡터 설정
    if elv_deg >= angle
        mode_hist(i) = 1; % Ground Station Mode
        target_vec_eci = r_rel_eci_m / norm(r_rel_eci_m); 
    else
        mode_hist(i) = 0; % Sun Pointing Mode
        target_vec_eci = rho_approx_hat(i, :).';
    end
    
    % Pointing Error 계산 (Body Frame 변환)
    C_bi = dcm_q(q_hist(i, :).'); % Quaternion to DCM
    target_vec_body = C_bi * target_vec_eci;
    
    dot_val = dot(target_vec_body, x_b);
    dot_val = max(min(dot_val, 1), -1); % Numerical stability
    pointing_err_deg(i) = acosd(dot_val);
end

%%
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
hold on;

% 1. 먼저 실제 데이터 그래프 그리기
plot(t_att_utc, rad2deg(w_hist), 'LineWidth', 1);

% 2. 강조할 구간 설정
t_detumble_end = t_att_utc(1) + seconds(Ts * 3); % 종료 시점 계산
y_lims = ylim; % 현재 그래프의 y축 위아래 끝값 가져오기

% 3. patch로 사각형 그리기 (시계 방향 또는 반시계 방향으로 꼭짓점 지정)
x_patch_detumble = [t_att_utc(1), t_detumble_end, t_detumble_end, t_att_utc(1)];
y_patch_detumble = [y_lims(1), y_lims(1), y_lims(2), y_lims(2)];

sun_idx = find(mode_hist == 1);
t_sunpoint_start = t_att_utc(sun_idx(1));
t_sunpoint_end = t_att_utc(sun_idx(end));

x_patch_sun = [t_sunpoint_start, t_sunpoint_end, t_sunpoint_end, t_sunpoint_start];
y_patch_sun = [y_lims(1), y_lims(1), y_lims(2), y_lims(2)];
p_1 = patch(x_patch_detumble, y_patch_detumble, 'b', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
p_2 = patch(x_patch_sun, y_patch_sun, 'y', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
uistack(p_1, 'bottom'); % 색칠된 영역을 그래프 뒤로 보내기
uistack(p_2, 'bottom');
grid on; 
ylabel('Ang Vel [deg/s]'); 
xlabel('Time [sec]');
legend('Ground Tracking Phase','Detumbling Phase','\omega_x', '\omega_y', '\omega_z' );
title('Body Angular Velocity');
hold off;
% (4) Quaternion History
figure('Name', 'Quaternion', 'Color', 'w');
plot(t_att_utc, q_hist, 'LineWidth', 1.2);
grid on; xlabel('Time [sec]'); ylabel('Quaternion');
legend('q_x', 'q_y', 'q_z', 'q_w (Scalar)');
title('Quaternion History');
%%
t_start = t_utc0;
figure('Name', 'Initial Detumbling Validation', 'Color', 'w');

% 1. 각속도 수렴 확인
subplot(2,1,1);
plot(t_att_utc, rad2deg(w_hist), 'LineWidth', 1.5); hold on;
% 설계된 Ts 지점에 가이드라인 표시
% t_start(datetime)에 seconds(Ts)(duration)를 더하면 결과는 datetime이 됩니다.
xline(t_start + seconds(Ts), '--r', ['Target Ts (', num2str(Ts), 's)'], 'LineWidth', 1.2);
yline(0, 'LineStyle','--');
grid on; 
ylabel('Ang Vel [deg/s]');
title('Initial Detumbling: Angular Velocity Convergence');
xlim([t_start, t_detumble_end]);
legend('\omega_x', '\omega_y', '\omega_z');

% 2. 포인팅 오차 수렴 확인 
subplot(2,1,2);
plot(t_att_utc, pointing_err_deg, 'k', 'LineWidth', 1.5); hold on;
xline(t_start + seconds(Ts), '--r', 'Target Ts');
% 초기 오차의 2% 라인 표시
yline(pointing_err_deg(1)*0.02, ':b', '2% Error Threshold');
grid on;
ylabel('Pointing Error [deg]');
xlabel('Time [UTC]');
title('Ground Mode : Pointing Error Convergence');
xlim([t_start ,  t_detumble_end]);
%%
startTime = t_att_utc(1);
stopTime = t_att_utc(end);

sampleTime = 1;


sc = satelliteScenario(startTime, stopTime, sampleTime);

step = max(1, round(sampleTime/dt));
idx = 1:step:length(t_att_utc);

pos_m = r_eci*1000;
vel_ms = v_eci*1000;

q_scalarFirst = [q_hist(:,4), q_hist(:,1:3)];
q_ds = q_scalarFirst(idx,:);

pos_m_ds = pos_m(idx, :);
vel_ms_ds = vel_ms(idx, :);
t_utc_ds = t_att_utc(idx);

positionTT = timetable(t_utc_ds(:), pos_m_ds,  'VariableNames', {'Position'});
velocityTT = timetable(t_utc_ds(:), vel_ms_ds, 'VariableNames', {'Velocity'});
attTT      = timetable(t_utc_ds(:), q_ds,      'VariableNames', {'q'});

% satellite
sat = satellite(sc, positionTT, velocityTT, ...
    'CoordinateFrame','inertial', 'Name','Mysat');

% attitude 먼저 적용
pointAt(sat, attTT, ...
    "CoordinateFrame","inertial", ...
    "Format","quaternion", ...
    "ExtrapolationMethod","fixed");

coordinateAxes(sat, Scale=2);

% ground station + access
gs = groundStation(sc, 'Name','INHA-AE', ...
    'Latitude', lat, 'Longitude', lon, 'Altitude', alt, ...
    'MinElevationAngle', angle);

ac = access(gs, sat);
ac.LineColor = 'green';

% ground track
groundTrack(sat);

% viewer & play
viewer = satelliteScenarioViewer(sc);
play(sc);
%%
itv = accessIntervals(ac);

disp(itv);
%%
fprintf('\nStarting Real-Time Animation...\n');

play_speed = 300; 
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
    gs_pos = r_gs_eci_m(idx,:)/1000;

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
    
    % (5) 화면 갱신 (핵심!)
    drawnow; 
end
fprintf('Animation Finished.\n');
