clc; clear all;
addpath('function');


%Parameter
mu = 398600;        % [km3/s2] Earth gravitational constant
R = 6371;           % [km] Earth radius
a = 600+R;          % [km] semi-major axis (SMA)(circle orbit)
e = 0;              % [-] eccentricity
inc = 97.8;         % [deg] inclination
raan = 0;           % [deg] 승교점이각
aop = 0;            % [deg] 근점이각
lat = 37.2302;      % [deg] 위도(항공우주산학융합원)
lon = 126.3925;     % [deg] 경도
alt = 0;            % [m] 해발고도
angle = 10;         % [deg] 최소교신각도

% Earth sphere
[xe, ye, ze] = sphere(80);  
Re = R;                      

xe = Re * xe;
ye = Re * ye;
ze = Re * ze;

%position 
r_pqw = [a; 0; 0];                    % km
v_pqw = [0; sqrt(mu/a); 0];          % km/s

%PQW to ECI rotation
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

%Time 
t_kst = datetime(2025,12,19,0,0,0,'TimeZone','Asia/Seoul');
t_utc0 = t_kst;
t_utc0.TimeZone = 'UTC';

%Two Body Dynamics -> 타원궤도가 되었을 때 타원파라미터를 쉽게 정의하기 위해서 타원식으로 전개
r0 = pqw2eci*r_pqw;
v0 = pqw2eci*v_pqw;
p = a*(1-e^2); 
b = a*sqrt(1-e^2);
T = 2*pi*sqrt(a^3/mu);
n = 2*pi/T;
h = sqrt(mu*p);
x0 = [r0; v0];
tspan = [0 1*T];
dt = 1;

%rigid dynamic
J  = diag([27.3, 48.2, 54.2]);
q0 = [0.2; -0.1; 0.3; 0.92]; 
q0 = q0/norm(q0);
w0 = deg2rad([1; -2; 0.5]);   % rad/s
x0_att = [q0; w0];

%Attitude control settings
qd = [0; 0; 0; 1];   
wd = [0;0;0];
Kp = diag([0.05, 0.05, 0.05]);   
Kd = diag([0.5,  0.5,  0.5 ]);   
tau_max = [inf; inf; inf];    

%simulate
[t, x] = rungekutta4(@(t,x) odeTwoBody(t,x,mu), tspan, x0, dt);
[t_att, x_att] = q_rungekutta4(@(t,x) attitude_ode_pd(t, x, J, qd, wd,Kp, Kd, tau_max), tspan, x0_att, dt);

%oribit results
r_eci = x(1:3, :).';
v_eci = x(4:6, :).';

%attitude results
q_hist = x_att(1:4,:).';
w_hist = x_att(5:7,:).';

%ECI to ECEF 
r_ecef = zeros(size(r_eci));
v_ecef = zeros(size(v_eci));

t_utc = t_utc0 + seconds(t);

for i = 1:length(t)
    utc_vec = datevec(t_utc(i));
    mjd = mjuliandate(utc_vec);
    pm = polarMotion(mjd);
    [re_m, ve_m] = eci2ecef(utc_vec, r_eci(i, :)*1000, v_eci(i, :)*1000, 'pm', pm);
    r_ecef(i,:) = re_m/1000;
    v_ecef(i,:) = ve_m/1000;

end

%ECI, ECEF Plot
figure; hold on;

surf(xe, ye, ze, ...
    'FaceColor',[0.6 0.8 1], ...
    'EdgeColor','none', ...
    'FaceAlpha',0.3, ...
    'DisplayName','Earth');

plot3(r_eci(:,1), r_eci(:,2), r_eci(:,3),'-o', ...
    'LineWidth',1.5, ...
    'DisplayName','ECI');

plot3(r_ecef(:,1), r_ecef(:,2), r_ecef(:,3),'-o', ...
    'LineWidth',1.5, ...
    'DisplayName','ECEF');

axis equal; grid on;
xlabel('X [km]'); ylabel('Y [km]'); zlabel('Z [km]');
title('Satellite trajectory');
legend('Location','best');

view(3);
rotate3d on;

%Sun postion 
jd = juliandate(t_utc);
jd = jd(:);

%라이브러리가 아닌 근사식 함수를 통한 태양 위치 계산
[r_sun_eci_approx, u_sun_eci_approx] = sun_eci_from_utc(t_utc);
rho_approx = r_sun_eci_approx - r_eci;
rho_approx_hat = rho_approx ./vecnorm(rho_approx,2,2);


%추가적인 ground track 궤적 확인
r_ecef_m = r_ecef*1e3;

lla = ecef2lla(r_ecef_m);
s_lat = lla(:,1);
s_lon = lla(:,2);
s_alt = lla(:,3);

s_lon = wrapTo180(s_lon);

figure
hold on;
plot(s_lon, s_lat, '--', 'LineWidth', 1.2, 'Color',"b", DisplayName="Trajectory")
scatter(lon, lat, DisplayName="Incheon")
grid on
xlabel('Longitude [deg]')
ylabel('Latitude [deg]')
title('Ground Track')
legend();
hold off;


figure; plot(t_att, w_hist); grid on;
xlabel('t [s]'); ylabel('\omega [rad/s]'); legend('p','q','r');






