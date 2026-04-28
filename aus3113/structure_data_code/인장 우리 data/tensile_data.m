clc, clear all;

%% data load %%
bambu_lateral  = readmatrix("Tensile_1.csv");
bambu_vertical = readmatrix("Tensile_2.csv");

cubicon_lateral  = readmatrix("Tensile_3.csv");
cubicon_vertical = readmatrix("Tensile_4.csv");

true_lateral  = readmatrix("tensile_1_v2.csv");
true_vertical = readmatrix("tensile_2_v2.csv");

%% strain %%
bambu_lateral_strain  = bambu_lateral(2:end,7)/100;
bambu_vertical_strain = bambu_vertical(2:end,7)/100;

cubicon_lateral_strain  = cubicon_lateral(2:end,7)/100;
cubicon_vertical_strain = cubicon_vertical(2:end,7)/100;

true_lateral_strain  = true_lateral(2:end,7)/100;
true_vertical_strain = true_vertical(2:end,7)/100;

%% stress %%
bambu_lateral_stress  = bambu_lateral(2:end,6)*1000;
bambu_vertical_stress = bambu_vertical(2:end,6)*1000;

cubicon_lateral_stress  = cubicon_lateral(2:end,6)*1000;
cubicon_vertical_stress = cubicon_vertical(2:end,6)*1000;

true_lateral_stress  = true_lateral(2:end,6)*1000;
true_vertical_stress = true_vertical(2:end,6)*1000;


%% ================================
%   최대하중
%% ================================
bambu_lateral_max_load  = max(bambu_lateral(2:end,3));
bambu_vertical_max_load = max(bambu_vertical(2:end,3));

cubicon_lateral_max_load  = max(cubicon_lateral(2:end,3));
cubicon_vertical_max_load = max(cubicon_vertical(2:end,3));

true_lateral_max_load   = max(true_lateral(2:end,3));
true_vertical_max_load  = max(true_vertical(2:end,3));

disp("=== 최대하중 [kN] ===")
fprintf("Bambu 0deg   : %.3f kN\n", bambu_lateral_max_load);
fprintf("Bambu 90deg  : %.3f kN\n", bambu_vertical_max_load);
fprintf("Cubicon 0deg : %.3f kN\n", cubicon_lateral_max_load);
fprintf("Cubicon 90deg: %.3f kN\n", cubicon_vertical_max_load);
fprintf("True 0deg    : %.3f kN\n", true_lateral_max_load);
fprintf("True 90deg   : %.3f kN\n\n", true_vertical_max_load);


%% ================================
%   연신률
%% ================================
bambu_lateral_elong  = max(bambu_lateral(2:end,4)) * 2;
bambu_vertical_elong = max(bambu_vertical(2:end,4)) * 2;

cubicon_lateral_elong  = max(cubicon_lateral(2:end,4)) * 2;
cubicon_vertical_elong = max(cubicon_vertical(2:end,4)) * 2;

true_lateral_elong   = max(true_lateral(2:end,4)) * 2;
true_vertical_elong  = max(true_vertical(2:end,4)) * 2;

disp("=== 연신률 [%] ===")
fprintf("Bambu 0deg   : %.2f %%\n", bambu_lateral_elong);
fprintf("Bambu 90deg  : %.2f %%\n", bambu_vertical_elong);
fprintf("Cubicon 0deg : %.2f %%\n", cubicon_lateral_elong);
fprintf("Cubicon 90deg: %.2f %%\n", cubicon_vertical_elong);
fprintf("True 0deg    : %.2f %%\n", true_lateral_elong);
fprintf("True 90deg   : %.2f %%\n\n", true_vertical_elong);


%% ================================
%   Young's Modulus (zero-intercept)
%% ================================
E_bambu_0   = compute_E_zero(bambu_lateral_strain,  bambu_lateral_stress, 0.005);
E_bambu_90  = compute_E_zero(bambu_vertical_strain, bambu_vertical_stress, 0.005);

E_cubi_0    = compute_E_zero(cubicon_lateral_strain,  cubicon_lateral_stress, 0.005);
E_cubi_90   = compute_E_zero(cubicon_vertical_strain, cubicon_vertical_stress, 0.005);

E_true_0    = compute_E_zero(true_lateral_strain,  true_lateral_stress, 0.006);
E_true_90   = compute_E_zero(true_vertical_strain, true_vertical_stress, 0.006);

disp("=== Young's Modulus (E) [MPa] ===")
fprintf("Bambu 0deg   : %.2f MPa\n", E_bambu_0);
fprintf("Bambu 90deg  : %.2f MPa\n", E_bambu_90);
fprintf("Cubicon 0deg : %.2f MPa\n", E_cubi_0);
fprintf("Cubicon 90deg: %.2f MPa\n", E_cubi_90);
fprintf("True 0deg    : %.2f MPa\n", E_true_0);
fprintf("True 90deg   : %.2f MPa\n\n", E_true_90);


%% ================================
%   최대 변위
%% ================================
bambu_lateral_max_length  = max(bambu_lateral(2:end,4));
bambu_vertical_max_length = max(bambu_vertical(2:end,4));

cubicon_lateral_max_length  = max(cubicon_lateral(2:end,4));
cubicon_vertical_max_length = max(cubicon_vertical(2:end,4));

true_lateral_max_length   = max(true_lateral(2:end,4));
true_vertical_max_length  = max(true_vertical(2:end,4));

disp("=== 최대 변위 [mm] ===")
fprintf("Bambu 0deg   : %.3f mm\n", bambu_lateral_max_length);
fprintf("Bambu 90deg  : %.3f mm\n", bambu_vertical_max_length);
fprintf("Cubicon 0deg : %.3f mm\n", cubicon_lateral_max_length);
fprintf("Cubicon 90deg: %.3f mm\n", cubicon_vertical_max_length);
fprintf("True 0deg    : %.3f mm\n", true_lateral_max_length);
fprintf("True 90deg   : %.3f mm\n\n", true_vertical_max_length);


%% ================================
%   최대 인장 강도
%% ================================
bambu_lateral_max_stress  = max(bambu_lateral_stress);
bambu_vertical_max_stress = max(bambu_vertical_stress);

cubicon_lateral_max_stress  = max(cubicon_lateral_stress);
cubicon_vertical_max_stress = max(cubicon_vertical_stress);

true_lateral_max_stress  = max(true_lateral_stress);
true_vertical_max_stress = max(true_vertical_stress);

disp("=== 최대 인장 강도 [MPa] ===")
fprintf("Bambu 0deg   : %.3f MPa\n", bambu_lateral_max_stress);
fprintf("Bambu 90deg  : %.3f MPa\n", bambu_vertical_max_stress);
fprintf("Cubicon 0deg : %.3f MPa\n", cubicon_lateral_max_stress);
fprintf("Cubicon 90deg: %.3f MPa\n", cubicon_vertical_max_stress);
fprintf("True 0deg    : %.3f MPa\n", true_lateral_max_stress);
fprintf("True 90deg   : %.3f MPa\n\n", true_vertical_max_stress);


%% ================================
%   Plot 
%% ================================
figure; hold on;

plot(bambu_lateral_strain,  bambu_lateral_stress,  'LineWidth',1.3,'DisplayName',"Bambu 0°");
plot(bambu_vertical_strain, bambu_vertical_stress, 'LineWidth',1.3,'DisplayName',"Bambu 90°");
plot(cubicon_lateral_strain,  cubicon_lateral_stress,  'LineWidth',1.3,'DisplayName',"Cubicon 0°");
plot(cubicon_vertical_strain, cubicon_vertical_stress, 'LineWidth',1.3,'DisplayName',"Cubicon 90°");
plot(true_lateral_strain,  true_lateral_stress,  'LineWidth',1.3,'DisplayName',"True 0°");
plot(true_vertical_strain, true_vertical_stress, 'LineWidth',1.3,'DisplayName',"True 90°");

xlabel("Strain[-]");
ylabel("Stress[{MPa}]");
title("Strain-Stress Curve");
legend();
grid on;
hold off;

%% ------------------------------------------------------------
function E = compute_E_zero(strain, stress, limit)
    valid = ~isnan(strain) & ~isnan(stress) & (strain <= limit);
    x = strain(valid);
    y = stress(valid);
    E = (x'*y) / (x'*x);
end
