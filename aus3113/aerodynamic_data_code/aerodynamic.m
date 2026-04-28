clc; clear all;

%모든 data의 순서는 -4,-2, 0,2,4,6,8이다.
aoa0_data = readmatrix("aoa0.txt");
aoa2_data = readmatrix("aoa2.txt");
aoa4_data = readmatrix("aoa4.txt");
aoa6_data = readmatrix("aoa6.txt");
aoa8_data = readmatrix("aoa8.txt");
aoam2_data = readmatrix("aoam2.txt");
aoam4_data = readmatrix("aoam4.txt");

data_list = [mean(aoam4_data); mean(aoam2_data); mean(aoa0_data); mean(aoa2_data); mean(aoa4_data); mean(aoa6_data); mean(aoa8_data); ];
angle_list = [-4, -2, 0, 2, 4, 6, 8 ];

%constant value
delta_P = [ 134.0,134.5,134.5, 134.2, 133.5, 131.4, 131.4];%Pa
Pair = 101300; %Pa
R_dry = 287.058; %J/kg/k
R_vapor = 461.495; %J/kg/k
chord_lendth = 0.2; %m
span_length = 0.5; %m
gamma = 1.4; %비열비
deg2rad = pi/180 ;

%processing data value
temp_data = zeros(7,1); %K
humid_data = zeros(7,1); %non dim
Fx = zeros(7,1); % N
Fy = zeros(7,1); % N 
Mz = zeros(7,1); % N
v = zeros(7,1); %m/s
rho_air = zeros(7,1); %kg/m^3
P_vapor = zeros(7,1); %Pa
Rair = zeros(7,1); %J/kg/k
mu = zeros(7,1); % non dim
speed_of_sound = zeros(7,1); %m/s
mach_number = zeros(7,1); % non dim
reynolds_number = zeros(7,1); 
Lift = zeros(7,1); % N
Drag = zeros(7,1); % N
CL = zeros(7,1); % non dim
CD = zeros(7,1); % non dim
Cm = zeros(7,1); % non dim




for i = 1:7 
    temp_data(i) = data_list(i, 10) + 273.15;
    humid_data(i) = data_list(i, 11)/100 ;
    Fx(i) = data_list(i, 1);
    Fy(i) = data_list(i, 2);
    Mz(i) = data_list(i, 6);
    P_vapor(i) =  humid_data(i) * 0.61078*exp(17.27*(temp_data(i)-273.15)/(temp_data(i)-35.85));
    rho_air(i) = (Pair-P_vapor(i))/R_dry/temp_data(i) + P_vapor(i)/R_vapor/temp_data(i);
    v(i) = sqrt((2*delta_P(i))/rho_air(i));
    Rair(i) = R_dry*(1- P_vapor(i)/Pair) + R_vapor*(P_vapor(i)/Pair);
    mu(i) = 1.716*10^(-5)*(temp_data(i)/273)^(2/3);
    speed_of_sound(i) = sqrt(gamma*Rair(i)*temp_data(i));
    mach_number(i) = v(i)/speed_of_sound(i);
    reynolds_number(i) = rho_air(i)*v(i)*chord_lendth/mu(i);
    Lift(i) = Fx(i)*sin(angle_list(i)*deg2rad) - Fy(i)*cos(angle_list(i)*deg2rad);
    Drag(i) = -Fx(i)*cos(angle_list(i)*deg2rad) - Fy(i)*sin(angle_list(i)*deg2rad);
    CL(i) = Lift(i)/(0.5*rho_air(i)*v(i)^2*chord_lendth*span_length);
    CD(i) = Drag(i)/(0.5*rho_air(i)*v(i)^2*chord_lendth*span_length);
    Cm(i) = -Mz(i)/(0.5*rho_air(i)*v(i)^2*chord_lendth^2*span_length);
end

figure();
hold on;
plot(angle_list, CL);
title("AOA vs C_{L}");
xlabel("AOA");
ylabel("C_{L}");
grid("on");
hold off;

figure();
hold on;
plot(CD, CL);
title("C_{D} vs C_{L}");
xlabel("C_{D}");
ylabel("C_{L}");
grid("on");
hold off;

figure();
hold on;
plot(angle_list, Cm);
title("AOA vs C_{m}");
xlabel("AOA");
ylabel("C_{m}");
grid("on");
hold off;

%%%
%받음각, CL, CD, CDp, CM, Top_Xtr, Bot_Xtr
naca_2412 = readmatrix("xf-naca2412-il-200000.csv");

% 1차 다항식(직선)으로 회귀, Least square 방식으로 직선을 찾는다.
poly_CL_alpha = polyfit(angle_list, CL, 1); 
CL_alpha_LS = polyval(poly_CL_alpha, angle_list);

CL_range = linspace(CL(1), CL(7), 100);
poly_CL_CD = polyfit(CL, CD, 2); 
CL_CD_LS = polyval(poly_CL_CD, CL_range);

figure();
hold on;
scatter(angle_list, CL, DisplayName="Data Point", Color='blue')
plot(angle_list, CL, DisplayName="Data", Color="blue", LineWidth=2);
plot(angle_list, CL_alpha_LS, DisplayName="Least Square", LineWidth=2);
title("AOA vs C_{L}")
xlabel("C_{D}");
ylabel("C_{L}");
grid("on");
legend();
hold off;

figure();
hold on;
scatter(angle_list, CL, DisplayName="Data Point", Color='blue')
plot(angle_list, CL, DisplayName="Data", Color="blue", LineWidth=2);
plot(angle_list, CL_alpha_LS, DisplayName="Least Square", LineWidth=2);
plot(naca_2412(:,1), naca_2412(:,2), DisplayName="NACA2412 DataBase", LineWidth=1);
title("AOA vs C_{L}")
xlabel("alpha");
ylabel("C_{L}");
grid("on");
legend();
hold off;


figure();
hold on;
scatter(CD, CL, DisplayName="Data Point", Color='blue')
plot(CD, CL, DisplayName="Data", Color="blue", LineWidth=2);
plot(CL_CD_LS, CL_range, DisplayName="Least Square", LineWidth=2, color ="red");
% plot(naca_2412(:,3), naca_2412(:,2), DisplayName="NACA2412 DataBase", LineWidth=1);
title("C_{D} vs C_{L}")
xlabel("C_{D}");
ylabel("C_{L}");
grid("on");
legend();
hold off;

figure();
hold on;
scatter(CD, CL, DisplayName="Data Point", Color='blue')
plot(CD, CL, DisplayName="Data", Color="blue", LineWidth=2);
plot(CL_CD_LS, CL_range, DisplayName="Least Square", LineWidth=2, color ="red");
plot(naca_2412(:,3), naca_2412(:,2), DisplayName="NACA2412 DataBase", LineWidth=1);
title("C_{D} vs C_{L}")
xlabel("C_{D}");
ylabel("C_{L}");
grid("on");
legend();
hold off;



figure();
hold on;
scatter(angle_list, Cm, DisplayName="Data Point", Color='blue')
plot(angle_list, Cm, DisplayName="Data", Color="blue", LineWidth=2);
plot(naca_2412(:,1), naca_2412(:,5), DisplayName="NACA2412 DataBase", LineWidth=1);
title("AOA vs C_{m}")
xlabel("AOA");
ylabel("C_{m}");
grid("on");
legend();
hold off;

