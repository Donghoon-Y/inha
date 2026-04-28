clc; clear all;

bending_lateral_raw_data = readmatrix("Bending_1.csv");
bending_vertical_raw_data = readmatrix("Bending_2.csv");

bending_lateral_filter_data = movmean(bending_lateral_raw_data, 50);
bending_vertical_filter_data = movmean(bending_vertical_raw_data, 50);

%%Strain%%
bending_lateral_raw_data_strain = bending_lateral_raw_data(2:end, 7)/100;
bending_vertical_raw_data_strain = bending_vertical_raw_data(2:end, 7)/100;
bending_lateral_filter_data_strain = bending_lateral_filter_data(2:end, 7)/100;
bending_vertical_filter_data_strain = bending_vertical_filter_data(2:end, 7)/100;

%%Stress%%
bending_lateral_raw_data_stress = bending_lateral_raw_data(2:end, 6)*1000;
bending_vertical_raw_data_stress = bending_vertical_raw_data(2:end, 6)*1000;
bending_lateral_filter_data_stress = bending_lateral_filter_data(2:end, 6)*1000;
bending_vertical_filter_data_stress = bending_vertical_filter_data(2:end, 6)*1000;

%%Strain-Stress Curve%%
figure();
hold on;
plot(bending_lateral_raw_data_strain, bending_lateral_raw_data_stress, DisplayName="raw 0deg");
plot(bending_vertical_raw_data_strain, bending_vertical_raw_data_stress, DisplayName="raw 90deg");
plot(bending_lateral_filter_data_strain, bending_lateral_filter_data_stress, DisplayName="filter 0deg", LineWidth=1.5);
plot(bending_vertical_filter_data_strain, bending_vertical_filter_data_stress, DisplayName="filter 90deg", LineWidth=1.5);
xlabel('Strain');
ylabel('Stress');
title('Strain-Stress Curve');
legend show;
grid on;
hold off;

%%구해야하는 Data%%

%%최대굽힘 강도%%
maxBendingStrength_lateral = max(bending_lateral_filter_data_stress);
maxBendingStrength_vertical = max(bending_vertical_filter_data_stress);

disp("====최대굽힘강도====");
fprintf("0deg : %.3f[MPa]\n", maxBendingStrength_lateral);
fprintf("90deg : %.3f[MPa]\n", maxBendingStrength_vertical);

%%최대하중%%
maxLoad_lateral =  max(bending_lateral_filter_data(2:end,3));
maxLoad_vertical = max(bending_vertical_filter_data(2:end,3));

disp("====최대하중====");
fprintf("0deg : %.3f[kN]\n", maxLoad_lateral);
fprintf("90deg : %.3f[kN]\n", maxLoad_vertical);

%%굽힘 변형률%%
max_bending_lateral_strain = max(bending_lateral_filter_data_strain)*100;
max_bending_vertical_strain = max(bending_vertical_filter_data_strain)*100;

disp("====굽힘 변형률====");
fprintf("0deg : %.3f\n", max_bending_lateral_strain);
fprintf("90deg : %.3f\n", max_bending_vertical_strain);

%%최대변위 종료값%%
max_bending_lateral_final_dis = max(bending_lateral_filter_data(2:end, 4));
max_bending_vertical_final_dis = max(bending_vertical_filter_data(2:end, 4));

disp("====최대변위 종료값====");
fprintf("0deg : %.3f[mm]\n", max_bending_lateral_final_dis);
fprintf("90deg : %.3f[mm]\n", max_bending_vertical_final_dis);

%%굽힘탄성계수%%
%%굽힘탄성계수%%
lateral_d = 3.38; lateral_b = 12.62; lateral_L = 54.08;
vertical_d = 3.61; vertical_b = 12.64; vertical_L = 57.76;

% Load [kN] -> [N]
bending_lateral_load_N  = bending_lateral_filter_data(2:end, 3) * 1000;
bending_vertical_load_N = bending_vertical_filter_data(2:end, 3) * 1000;

% Displacement [mm]
bending_lateral_dis  = bending_lateral_filter_data(2:end, 4);
bending_vertical_dis = bending_vertical_filter_data(2:end, 4);


% ---- (1) 최대 하중의 10~30% 구간 선택 ----
lat_Pmax = max(bending_lateral_load_N);
ver_Pmax = max(bending_vertical_load_N);

lat_idx = (bending_lateral_load_N  >= 0.10*lat_Pmax) & ...
          (bending_lateral_load_N  <= 0.30*lat_Pmax);
ver_idx = (bending_vertical_load_N >= 0.30*ver_Pmax) & ...
          (bending_vertical_load_N <= 0.50*ver_Pmax);

x_lat = bending_lateral_dis(lat_idx);
y_lat = bending_lateral_load_N(lat_idx);
m_lateral = (x_lat' * y_lat) / (x_lat' * x_lat);     % [N/mm]

x_ver = bending_vertical_dis(ver_idx);
y_ver = bending_vertical_load_N(ver_idx);
m_vertical = (x_ver' * y_ver) / (x_ver' * x_ver);    % [N/mm]

% ---- (3) ASTM D790 공식으로 굽힘탄성계수 계산 ----
E_f_lateral  = (lateral_L^3  * m_lateral ) / (4 * lateral_b  * lateral_d^3);
E_f_vertical = (vertical_L^3 * m_vertical) / (4 * vertical_b * vertical_d^3);

disp("==== 굽힘 탄성계수 (Flexural Modulus) [MPa] ====");
fprintf("0deg : %.2f MPa (m = %.3f N/mm)\n", E_f_lateral,  m_lateral);
fprintf("90deg: %.2f MPa (m = %.3f N/mm)\n", E_f_vertical, m_vertical);
