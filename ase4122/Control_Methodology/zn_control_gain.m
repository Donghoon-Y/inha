%% ============================================================
%  Quadrotor Roll Channel Linearization + Root Locus + ZN PID
%  Based on Luukkonen (2011)
%% ============================================================
clear; clc; close all;

%% -- 1. Parameters ------------------------------------------
% Luukkonen Table 1 (replace with CAD values after structure finalized)
Ixx = 4.856e-3;   % [kg*m^2] Roll moment of inertia
Iyy = 4.856e-3;   % [kg*m^2] Pitch moment of inertia
l   = 0.225;      % [m]      Arm length (rotor to CoM)
k   = 2.980e-6;   % Lift constant
b   = 1.140e-7;   % Drag constant

fprintf('=== Parameters ===\n');
fprintf('  Ixx = %.4e kg*m^2\n', Ixx);
fprintf('  l   = %.4f m\n', l);
fprintf('  k   = %.4e\n', k);

%% -- 2. Transfer Function G(s) = 1/(Ixx*s^2) ----------------
% Input : tau_phi [N*m]
% Output: phi [rad]
% Linearized: phi_ddot = tau_phi / Ixx
% Laplace   : G(s) = 1 / (Ixx * s^2)
num_G = 1;
den_G = [Ixx 0 0];
G = tf(num_G, den_G);

fprintf('\n=== Transfer Function G(s) ===\n');
display(G);

%% -- 3. Root Locus: P control only --------------------------
% Double pole at origin -> locus stays on imaginary axis
% -> Cannot stabilize with P only -> Need Kd (zero insertion)
figure('Name','Root Locus - P control only','NumberTitle','off');
rlocus(G);
title('Root Locus: P control only  G = 1/(Ixx*s^2)');
xlabel('Real Axis');
ylabel('Imaginary Axis');
grid on;
hold on;

zeta_target = 0.707;   % 45 degree damping ratio line
r_line = 10;
plot([0, -r_line*zeta_target], [0,  r_line*sqrt(1-zeta_target^2)], ...
    'r--', 'LineWidth', 1.5, 'DisplayName','zeta=0.707 (45 deg)');
plot([0, -r_line*zeta_target], [0, -r_line*sqrt(1-zeta_target^2)], ...
    'r--', 'LineWidth', 1.5, 'HandleVisibility','off');
legend show;
hold off;

fprintf('\n[NOTE] P only: locus stays on imaginary axis -> Add Kd\n');

%% -- 4. PD Control: insert zero to bend locus ---------------
% PD controller: C(s) = Kd_init*s + 1
% G_PD(s) = C(s)*G(s) = (Kd_init*s + 1) / (Ixx*s^2)
% Zero location: s = -1/Kd_init
%
% Target settling time ~2s -> wn ~ 3-5 rad/s
% Place zero near s = -3 -> Kd_init ~ 1/3 ~ 0.33
Kd_init = 0.33;

C_PD = tf([Kd_init 1], 1);
G_PD = C_PD * G;

figure('Name','Root Locus - PD control','NumberTitle','off');
rlocus(G_PD);
title(sprintf('Root Locus: PD control  Kd=%.2f  zero @ s=%.2f', ...
    Kd_init, -1/Kd_init));
xlabel('Real Axis');
ylabel('Imaginary Axis');
grid on;
hold on;

r_line = 20;
plot([0, -r_line*zeta_target], [0,  r_line*sqrt(1-zeta_target^2)], ...
    'r--', 'LineWidth', 1.5, 'DisplayName','zeta=0.707 (45 deg)');
plot([0, -r_line*zeta_target], [0, -r_line*sqrt(1-zeta_target^2)], ...
    'r--', 'LineWidth', 1.5, 'HandleVisibility','off');
legend show;
hold off;

%% -- 5. Extract Ku and Tu via Bode Plot ----------------------
fprintf('\n=== Bode Plot -> Ku, Tu ===\n');

figure('Name','Bode Plot - G_PD','NumberTitle','off');
margin(G_PD);
title(sprintf('Bode Plot: G\\_PD(s)  Kd=%.2f', Kd_init));
grid on;

[Gm, Pm, Wcg, Wcp] = margin(G_PD);

fprintf('  Gain  Margin = %.4f  (%.2f dB)\n', Gm, 20*log10(max(Gm,1e-9)));
fprintf('  Phase Margin = %.2f deg\n', Pm);
fprintf('  Phase crossover freq (wu) = %.4f rad/s\n', Wcg);
fprintf('  Gain  crossover freq (wgc)= %.4f rad/s\n', Wcp);

if isfinite(Wcg) && Wcg > 0
    wu = Wcg;
    Ku = Gm;
    Tu = 2*pi / wu;
    fprintf('\n  -> wu = %.4f rad/s\n', wu);
    fprintf('  -> Ku = %.4f\n', Ku);
    fprintf('  -> Tu = 2*pi/wu = %.4f s\n', Tu);
else
    % Gain Margin = Inf case (common for double integrator + PD zero)
    % -> Set Ku directly from desired closed-loop wn
    fprintf('\n  [Gain Margin = Inf] -> Using target wn instead\n');
    wn_target = 4.0;   % desired natural freq [rad/s], tune as needed
    Ku = Ixx * wn_target^2;
    Tu = 2*pi / wn_target;
    fprintf('  Target wn = %.1f rad/s\n', wn_target);
    fprintf('  -> Ku = Ixx * wn^2 = %.4f\n', Ku);
    fprintf('  -> Tu = 2*pi / wn  = %.4f s\n', Tu);
end

%% -- 6. Ziegler-Nichols PID Initial Gains --------------------
Kp_ZN = 0.6   * Ku;
Ki_ZN = Kp_ZN / (0.5 * Tu);
Kd_ZN = Kp_ZN * (0.125 * Tu);

fprintf('\n=== ZN PID Initial Gains (Roll channel) ===\n');
fprintf('  Kp = 0.600 * Ku           = %.4f\n', Kp_ZN);
fprintf('  Ki = Kp / (0.5 * Tu)      = %.4f\n', Ki_ZN);
fprintf('  Kd = Kp * (0.125 * Tu)    = %.4f\n', Kd_ZN);

%% -- 7. Closed-loop Step Response with ZN Gains --------------
C_PID = pid(Kp_ZN, Ki_ZN, Kd_ZN);
T_cl  = feedback(C_PID * G, 1);

figure('Name','Step Response - ZN PID','NumberTitle','off');
step(T_cl, 5);
title('Closed-loop Step Response  (ZN initial gains)');
xlabel('Time [s]');
ylabel('Roll phi [rad / rad\_ref]');
grid on;

info = stepinfo(T_cl);
fprintf('\n=== Step Response Metrics ===\n');
fprintf('  Overshoot     = %.2f %%\n', info.Overshoot);
fprintf('  Settling time = %.4f s\n',  info.SettlingTime);
fprintf('  Rise time     = %.4f s\n',  info.RiseTime);
fprintf('  Peak          = %.4f\n',    info.Peak);

%% -- 8. Pitch channel (Iyy ~ Ixx -> same gains) -------------
fprintf('\n=== Pitch Channel ===\n');
diff_pct = abs(Ixx - Iyy) / Ixx * 100;
if diff_pct < 1.0
    fprintf('  Ixx ~ Iyy (diff = %.2f%%) -> same gains applicable\n', diff_pct);
    fprintf('  Kp_pitch = %.4f\n', Kp_ZN);
    fprintf('  Ki_pitch = %.4f\n', Ki_ZN);
    fprintf('  Kd_pitch = %.4f\n', Kd_ZN);
else
    fprintf('  Ixx != Iyy -> design separately\n');
end

%% -- 9. Summary for Gazebo SILS Warm Start ------------------
fprintf('\n========================================\n');
fprintf('  Gazebo SILS Warm Start Summary\n');
fprintf('========================================\n');
fprintf('  [Roll / Pitch]\n');
fprintf('    Kp = %.4f\n', Kp_ZN);
fprintf('    Ki = %.4f\n', Ki_ZN);
fprintf('    Kd = %.4f\n', Kd_ZN);
fprintf('\n');
fprintf('  NOTE: Replace Ixx with CAD value after structure lock\n');
fprintf('  TARGET: overshoot < 20%%,  settling < 2 s\n');
fprintf('  TUNE:   overshoot high  -> decrease Kp or increase Kd\n');
fprintf('          too slow        -> increase Kp\n');
fprintf('          steady-state err-> increase Ki slowly\n');
fprintf('========================================\n');