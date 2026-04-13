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
       sin(deg2rad(raan))  cos(deg2rad(raan)) 0;
       0 0 1];
R_i = [1 0 0;
       0  cos(deg2rad(inc)) -sin(deg2rad(inc));
       0  sin(deg2rad(inc))  cos(deg2rad(inc))];
R_w = [cos(deg2rad(aop)) -sin(deg2rad(aop)) 0;
       sin(deg2rad(aop))  cos(deg2rad(aop)) 0;
       0 0 1];
pqw2eci = R_W * R_i * R_w;

% Time Settings
t_kst  = datetime(2023,12,19,0,0,0,'TimeZone','Asia/Seoul');
t_utc0 = t_kst;
t_utc0.TimeZone = 'UTC';

% Orbit Simulation (Two Body)
r0 = pqw2eci * r_pqw;
v0 = pqw2eci * v_pqw;
T_period = 2*pi*sqrt(a^3/mu);
x0    = [r0; v0];
tspan = [0 5*T_period];
dt    = 0.5;

fprintf("Orbit Simulation Start.......\n");
[t, x] = rungekutta4(@(t,x) odeTwoBody(t,x,mu), tspan, x0, dt);

% Orbit results
r_eci = x(1:3, :).';  % [km] N x 3
v_eci = x(4:6, :).';  % [km/s] N x 3

% ECI to ECEF Conversion (for Ground Track)
r_ecef = zeros(size(r_eci));
v_ecef = zeros(size(v_eci));
t_utc  = t_utc0 + seconds(t);
cnt    = 0;

fprintf("Converting ECI to ECEF.......\n");
for i = 1:length(t)
    utc_vec = datevec(t_utc(i));
    mjd = mjuliandate(utc_vec);
    [re_m, ve_m] = eci2ecef(utc_vec, r_eci(i,:)*1000, v_eci(i,:)*1000);
    r_ecef(i,:) = re_m / 1000;
    v_ecef(i,:) = ve_m / 1000;
    if mod(i, 500) == 0
        fprintf('%d ', i);
        cnt = cnt + 1;
        if mod(cnt, 10) == 0, fprintf('\n'); end
    end
end
fprintf('\n');
disp("Orbit Calculation Complete.");

%% Sun Position
[r_sun_eci_approx, ~] = sun_eci_from_utc(t_utc);
rho_approx     = r_sun_eci_approx - r_eci;
rho_approx_hat = rho_approx ./ vecnorm(rho_approx, 2, 2);

%% Rigid Body Parameters
J  = diag([0.070, 0.071, 0.007]);
q0 = [0.2; -0.1; 0.3; 0.92];
q0 = q0 / norm(q0);
w0 = deg2rad([1; -2; 0.5]);
x0_att = [q0; w0];

% Controller Gains
Ts      = 40;
zeta    = 1/sqrt(2);
wn      = 4/(Ts*zeta);
Kp      = diag(diag(J) * wn^2);
Kd      = diag(2*zeta*diag(J)*wn);
tau_max = [0.1*1e-3; 0.1*1e-3; 0.1*1e-3];

%% Ground Station ECI 변환
fprintf("Attitude Simulation Start (Switching Mode).......\n");

r_gs_ecef_m = x_g_ecef * 1000;
point       = length(t_utc);
r_gs_eci_m  = zeros(point, 3);

for i = 1:point
    r_temp = ecef2eci(t_utc(i), r_gs_ecef_m);
    r_gs_eci_m(i,:) = r_temp;
    if mod(i, 1000) == 0
        fprintf('GS ECI Progress: %d / %d (%.1f%%)\n', i, point, (i/point)*100);
    end
end

%% Attitude Simulation
% q_rungekutta4에는 dx만 반환하는 래퍼 사용
% (attitude_mode_switch가 [dx, w_des_out] 두 개를 반환하므로)
ode_wrapper = @(tt, xx) get_dx(tt, xx, J, Kp, Kd, tau_max, ...
                               r_eci, v_eci, rho_approx_hat, ...
                               r_gs_ecef_m, angle, dt, t_utc0);

[t_att, x_att] = q_rungekutta4(ode_wrapper, tspan, x0_att, dt);

q_hist    = x_att(1:4, :).';
w_hist    = x_att(5:7, :).';
t_att_utc = t_utc0 + seconds(t_att);

N               = length(t_att);
pointing_err_deg = zeros(N, 1);
mode_hist        = zeros(N, 1);   % 0: Sun,  1: Ground
w_des_hist       = zeros(N, 3);   % ← 추가: 목표 각속도 (ECI) [rad/s]

x_b = [1; 0; 0];  % Body X-axis (Antenna)

fprintf("Analyzing Results.......\n");
for i = 1:N
    r_sat_eci_m    = r_eci(i,:).' * 1000;
    r_gs_now_eci_m = r_gs_eci_m(i,:).';

    % 고도각 계산
    r_rel_eci_m = r_gs_now_eci_m - r_sat_eci_m;
    u_zenith    = r_gs_now_eci_m / norm(r_gs_now_eci_m);
    u_gs2sat    = -r_rel_eci_m / norm(r_rel_eci_m);
    elv_deg     = rad2deg(asin(dot(u_gs2sat, u_zenith)));

    if elv_deg >= angle
        %% GS 모드
        mode_hist(i)   = 1;
        target_vec_eci = r_rel_eci_m / norm(r_rel_eci_m);

        % w_des 재계산 (attitude_mode_switch 내부 로직과 동일)
        t_current = t_utc0 + seconds(t_att(i));
        v_sat_m   = v_eci(i,:).' * 1000;
        r_gs_now  = ecef2eci(t_current, r_gs_ecef_m);
        We_vec    = [0; 0; 7.292115e-5];
        v_gs_now  = cross(We_vec, r_gs_now);
        v_rel     = v_gs_now - v_sat_m;
        u_vec     = r_rel_eci_m / norm(r_rel_eci_m);
        w_des_hist(i,:) = (cross(u_vec, v_rel) / norm(r_rel_eci_m)).';
    else
        %% Sun 모드
        mode_hist(i)    = 0;
        target_vec_eci  = rho_approx_hat(i,:).';
        w_des_hist(i,:) = [0, 0, 0];   % Sun 모드 목표 각속도 = 0
    end

    % Pointing Error
    C_bi            = dcm_q(q_hist(i,:).');
    target_vec_body = C_bi * target_vec_eci;
    dot_val         = max(min(dot(target_vec_body, x_b), 1), -1);
    pointing_err_deg(i) = acosd(dot_val);
end

%% Plot

% (1) 3D Orbit
figure('Name','3D Orbit','Color','w'); hold on;
surf(xe, ye, ze, 'FaceColor',[0.6 0.8 1], 'EdgeColor','none', ...
     'FaceAlpha',0.3, 'DisplayName','Earth');
plot3(r_eci(:,1), r_eci(:,2), r_eci(:,3), '-b', 'LineWidth',1, 'DisplayName','Orbit');
plot3(x_g_ecef(1), x_g_ecef(2), x_g_ecef(3), 'rp', ...
      'MarkerSize',10, 'MarkerFaceColor','r', 'DisplayName','GS (t=0)');
axis equal; grid on;
xlabel('X [km]'); ylabel('Y [km]'); zlabel('Z [km]');
legend; view(3); title('Satellite Trajectory (ECI)');

% (2) Ground Track
r_ecef_m = r_ecef * 1e3;
lla      = ecef2lla(r_ecef_m);
s_lat    = lla(:,1);
s_lon    = wrapTo180(lla(:,2));
figure('Name','Ground Track','Color','w'); hold on;
plot(s_lon, s_lat, '.', 'MarkerSize',2, 'Color','b', 'DisplayName','Trajectory');
scatter(lon, lat, 50, 'r', 'filled', 'DisplayName','Incheon GS');
grid on; xlabel('Longitude [deg]'); ylabel('Latitude [deg]');
title('Ground Track'); legend; xlim([-180 180]); ylim([-90 90]);

% (3) Control Performance
figure('Name','Control Performance','Color','w');

subplot(3,1,1);
area(t_att_utc, mode_hist, 'FaceColor',[0.2 0.8 0.2], 'FaceAlpha',0.3);
ylim([-0.1 1.1]); yticks([0 1]); yticklabels({'Sun','Ground'});
grid on; ylabel('Mode');
title('Attitude Control Mode Switching');

subplot(3,1,2);
plot(t_att_utc, pointing_err_deg, 'LineWidth',1.2, 'Color','k');
grid on; ylabel('Error [deg]');
title('Pointing Error (Relative to Active Target)');
yline(0,'--r');
ylim([0, max(10, max(pointing_err_deg)*1.1)]);

subplot(3,1,3); hold on;
plot(t_att_utc, rad2deg(w_hist), 'LineWidth',1);
t_detumble_end = t_att_utc(1) + seconds(Ts*3);
y_lims = ylim;
x_patch_detumble = [t_att_utc(1), t_detumble_end, t_detumble_end, t_att_utc(1)];
y_patch_detumble = [y_lims(1), y_lims(1), y_lims(2), y_lims(2)];
sun_idx = find(mode_hist == 1);
if ~isempty(sun_idx)
    t_sunpoint_start = t_att_utc(sun_idx(1));
    t_sunpoint_end   = t_att_utc(sun_idx(end));
    x_patch_sun = [t_sunpoint_start, t_sunpoint_end, t_sunpoint_end, t_sunpoint_start];
    y_patch_sun = [y_lims(1), y_lims(1), y_lims(2), y_lims(2)];
    p_2 = patch(x_patch_sun, y_patch_sun, 'y', 'FaceAlpha',0.3, 'EdgeColor','none');
    uistack(p_2,'bottom');
end
p_1 = patch(x_patch_detumble, y_patch_detumble, 'b', 'FaceAlpha',0.3, 'EdgeColor','none');
uistack(p_1,'bottom');
grid on; ylabel('Ang Vel [deg/s]'); xlabel('Time');
legend('Ground Tracking Phase','Detumbling Phase','\omega_x','\omega_y','\omega_z');
title('Body Angular Velocity'); hold off;

% (4) Quaternion History
figure('Name','Quaternion','Color','w');
plot(t_att_utc, q_hist, 'LineWidth',1.2);
grid on; xlabel('Time'); ylabel('Quaternion');
legend('q_x','q_y','q_z','q_w (Scalar)');
title('Quaternion History');

% (5) Detumbling Validation
t_start = t_utc0;
figure('Name','Initial Detumbling Validation','Color','w');

subplot(2,1,1);
plot(t_att_utc, rad2deg(w_hist), 'LineWidth',1.5); hold on;
xline(t_start + seconds(Ts), '--r', ['Target Ts (', num2str(Ts), 's)'], 'LineWidth',1.2);
yline(0,'LineStyle','--');
grid on; ylabel('Ang Vel [deg/s]');
title('Initial Detumbling: Angular Velocity Convergence');
xlim([t_start, t_detumble_end]);
legend('\omega_x','\omega_y','\omega_z');

subplot(2,1,2);
plot(t_att_utc, pointing_err_deg, 'k', 'LineWidth',1.5); hold on;
xline(t_start + seconds(Ts), '--r', 'Target Ts');
yline(pointing_err_deg(1)*0.02, ':b', '2% Error Threshold');
grid on; ylabel('Pointing Error [deg]'); xlabel('Time [UTC]');
title('Ground Mode: Pointing Error Convergence');
xlim([t_start, t_detumble_end]);

%% Satellite Scenario Viewer
startTime  = t_att_utc(1);
stopTime   = t_att_utc(end);
sampleTime = 1;

sc   = satelliteScenario(startTime, stopTime, sampleTime);
step = max(1, round(sampleTime/dt));
idx  = 1:step:length(t_att_utc);

pos_m    = r_eci * 1000;
vel_ms   = v_eci * 1000;
q_scalarFirst = [q_hist(:,4), q_hist(:,1:3)];

pos_m_ds  = pos_m(idx,:);
vel_ms_ds = vel_ms(idx,:);
q_ds      = q_scalarFirst(idx,:);
t_utc_ds  = t_att_utc(idx);

positionTT = timetable(t_utc_ds(:), pos_m_ds,  'VariableNames',{'Position'});
velocityTT = timetable(t_utc_ds(:), vel_ms_ds, 'VariableNames',{'Velocity'});
attTT      = timetable(t_utc_ds(:), q_ds,      'VariableNames',{'q'});

sat = satellite(sc, positionTT, velocityTT, ...
    'CoordinateFrame','inertial', 'Name','Mysat');
pointAt(sat, attTT, ...
    'CoordinateFrame','inertial', ...
    'Format','quaternion', ...
    'ExtrapolationMethod','fixed');
coordinateAxes(sat, Scale=2);

gs = groundStation(sc, 'Name','INHA-AE', ...
    'Latitude',lat, 'Longitude',lon, 'Altitude',alt, ...
    'MinElevationAngle',angle);
ac = access(gs, sat);
ac.LineColor = 'green';
groundTrack(sat);

viewer = satelliteScenarioViewer(sc);
play(sc);

%% CSV Export
fprintf("Exporting simulation data to CSV...\n");

t_export = t_att(:);

% 열 구성 (총 21열):
% [시간(1), 쿼터니언(4), 각속도(3), 태양벡터(3), 위성위치(3), 지상국위치(3), 모드(1), w_des(3)]
export_data = [t_export, q_hist, w_hist, rho_approx_hat, r_eci, r_gs_eci_m, mode_hist, w_des_hist];

file_name = 'my_satellite_data.csv';
writematrix(export_data, file_name);

fprintf("Data successfully saved to %s\n", file_name);
fprintf("- Total Samples : %d\n", N);
fprintf("- Total Columns : %d\n", size(export_data, 2));
fprintf("\nCSV Column Layout:\n");
fprintf("  Col  1    : time [s]\n");
fprintf("  Col  2- 5 : quaternion [qx qy qz qw]\n");
fprintf("  Col  6- 8 : angular velocity [rad/s]\n");
fprintf("  Col  9-11 : sun vector ECI (unit)\n");
fprintf("  Col 12-14 : satellite position ECI [km]\n");
fprintf("  Col 15-17 : ground station position ECI [m]\n");
fprintf("  Col 18    : mode [0=Sun, 1=GS]\n");
fprintf("  Col 19-21 : w_des ECI [rad/s]\n");