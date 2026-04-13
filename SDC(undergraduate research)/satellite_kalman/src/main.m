clc; 
clear all;
thisFile = mfilename('fullpath');
thisDir  = fileparts(thisFile);
addpath(genpath(thisDir));

%% 1. 데이터 로드
fprintf("Data loading...\n");
true_data   = readmatrix("my_satellite_data.csv");
t_sim       = true_data(:, 1);
q_true      = true_data(:, 2:5);
w_true      = true_data(:, 6:8);
sun_eci_ref = true_data(:, 9:11);
r_sat_eci   = true_data(:, 12:14);
r_gs_eci    = true_data(:, 15:17);
mode        = true_data(:, 18);
w_des       = true_data(:, 19:21);

N  = length(t_sim);
dt = t_sim(2) - t_sim(1);

fprintf('dt = %.4f sec\n', dt);
fprintf('Sun mode: %d steps (%.1f%%)\n', sum(mode==0), sum(mode==0)/N*100);
fprintf('GS  mode: %d steps (%.1f%%)\n', sum(mode==1), sum(mode==1)/N*100);

%% 2. 센서 노이즈 및 랜덤 워크 바이어스 생성
arw = 0.001;
rrw = 1e-5;
true_bias      = zeros(N, 3);
true_bias(1,:) = [0.01, -0.01, 0.005];
for k = 2:N
    true_bias(k,:) = true_bias(k-1,:) + randn(1,3) * rrw * sqrt(dt);
end
w_meas = w_true + true_bias + randn(N,3) * arw;

%% 3. EKF 초기화
x_est = [q_true(1,:)'; 0; 0; 0];
P = diag([1e-4*ones(1,4), 1e-4*ones(1,3)]);

q_quat_nominal = 1e-6;
q_quat_switch  = 1e-3;
q_bias         = rrw^2;

R_sun   = eye(3) * deg2rad(1.0)^2;
R_nadir = eye(3) * deg2rad(2.0)^2;
R_gs    = eye(3) * deg2rad(0.5)^2;

x_hist        = zeros(N, 7);
P_hist        = zeros(N, 7);
R_gs_eff_hist = zeros(N, 1);

fprintf("EKF Estimation Start...\n");
cnt = 0;
mode_prev = mode(1);

%% 4. EKF Loop
for k = 1:N

    %% --- 모드 전환 감지 ---
    mode_changed = (k > 1) && (mode(k) ~= mode_prev);
    mode_prev    = mode(k);

    %% --- Q 및 threshold 동적 설정 ---
    if mode_changed
        q_quat_curr    = q_quat_switch;
        thresh_primary = deg2rad(60);
        thresh_nadir   = deg2rad(60);   % Nadir threshold 완화
        fprintf('  [Mode Switch] k=%d, t=%.1fs, %s -> %s\n', ...
            k, t_sim(k), mode_str(mode(k-1)), mode_str(mode(k)));
    else
        q_quat_curr    = q_quat_nominal;
        thresh_primary = deg2rad(30);
        thresh_nadir   = deg2rad(30);
    end
    Q_curr = diag([q_quat_curr*ones(1,4), q_bias*ones(1,3)]);

    %% --- Prediction ---
    w_curr = w_meas(k,:)' - x_est(5:7);

    Omega = [ 0,          w_curr(3), -w_curr(2),  w_curr(1);
             -w_curr(3),  0,          w_curr(1),  w_curr(2);
              w_curr(2), -w_curr(1),  0,           w_curr(3);
             -w_curr(1), -w_curr(2), -w_curr(3),  0         ];

    theta = norm(w_curr) * dt;

    if theta > 1e-12
        exp_Omega  = eye(4)*cos(0.5*theta) + Omega*(sin(0.5*theta)/norm(w_curr));
        x_est(1:4) = exp_Omega * x_est(1:4);
    else
        exp_Omega  = eye(4) + 0.5*Omega*dt;
        x_est(1:4) = x_est(1:4) + 0.5*Omega*x_est(1:4)*dt;
    end
    x_est(1:4) = x_est(1:4) / norm(x_est(1:4));

    q   = x_est(1:4);
    Xi  = [ q(4), -q(3),  q(2);
            q(3),  q(4), -q(1);
           -q(2),  q(1),  q(4);
           -q(1), -q(2), -q(3)];

    F          = eye(7);
    F(1:4,1:4) = exp_Omega;
    F(1:4,5:7) = -0.5 * Xi * dt;

    P = F * P * F' + Q_curr;

    %% --- Correction ---
    C_est  = dcm_q(x_est(1:4));
    C_true = dcm_q(q_true(k,:));

    v_nadir = -r_sat_eci(k,:)';
    v_nadir = v_nadir / norm(v_nadir);

    if mode(k) == 0
        %% --- Sun 모드 ---
        v_sun = sun_eci_ref(k,:)';
        v_sun = v_sun / norm(v_sun);

        z_meas = [C_true * v_sun   + randn(3,1)*deg2rad(1.0);
                  C_true * v_nadir + randn(3,1)*deg2rad(2.0)];
        z_hat  = [C_est  * v_sun;
                  C_est  * v_nadir];

        H = [jacobian_h_q(x_est(1:4), v_sun),   zeros(3,3);
             jacobian_h_q(x_est(1:4), v_nadir),  zeros(3,3)];

        % Sun 모드 전환 시 Nadir R도 키움
        if mode_changed
            R_curr = blkdiag(R_sun * 4, R_nadir * 10);
        else
            R_curr = blkdiag(R_sun, R_nadir);
        end

    else
        %% --- GS 모드 ---
        v_gs = (r_gs_eci(k,:) - r_sat_eci(k,:))';
        v_gs = v_gs / norm(v_gs);

        z_meas = [C_true * v_gs    + randn(3,1)*deg2rad(0.5);
                  C_true * v_nadir + randn(3,1)*deg2rad(2.0)];
        z_hat  = [C_est  * v_gs;
                  C_est  * v_nadir];

        H = [jacobian_h_q(x_est(1:4), v_gs),    zeros(3,3);
             jacobian_h_q(x_est(1:4), v_nadir),  zeros(3,3)];

        % 방법 2: v_gs 변화율 → R_gs에 추가
        w_des_k    = w_des(k,:)';
        v_gs_dot   = cross(w_des_k, v_gs);
        delta_ang  = norm(v_gs_dot) * dt;
        R_gs_extra = eye(3) * delta_ang^2;
        R_gs_eff   = R_gs + R_gs_extra;
        R_gs_eff_hist(k) = rad2deg(sqrt(R_gs_eff(1,1)));

        % GS 모드 전환 시 Nadir R도 키움
        if mode_changed
            R_curr = blkdiag(R_gs_eff * 4, R_nadir * 10);
        else
            R_curr = blkdiag(R_gs_eff, R_nadir);
        end
    end

    %% --- Outlier Rejection (primary / Nadir 따로) ---
    innov = z_meas - z_hat;

    if mode_changed
        fprintf('  [Switch k=%d] innov(1:3)=%.3f deg, innov(4:6)=%.3f deg\n', ...
            k, rad2deg(norm(innov(1:3))), rad2deg(norm(innov(4:6))));
    end

    if norm(innov(1:3)) > thresh_primary || norm(innov(4:6)) > thresh_nadir
        x_hist(k,:) = x_est';
        P_hist(k,:) = diag(P)';
        continue;
    end

    %% --- 칼만 게인 ---
    S = H * P * H' + R_curr;
    K = P * H' / S;

    %% --- 상태 업데이트 ---
    x_est      = x_est + K * innov;
    x_est(1:4) = x_est(1:4) / norm(x_est(1:4));

    %% --- P 업데이트 (Joseph form) ---
    IKH = eye(7) - K * H;
    P   = IKH * P * IKH' + K * R_curr * K';
    P   = (P + P') / 2;

    x_hist(k,:) = x_est';
    P_hist(k,:) = diag(P)';

    if mod(k, 500) == 0
        fprintf('%d ', k);
        cnt = cnt + 1;
        if mod(cnt, 10) == 0, fprintf('\n'); end
    end
end
fprintf('\nEKF Calculation Complete.\n');

%% 5. 각도 오차 계산
angle_error_deg = zeros(N, 1);
for k = 1:N
    q_t     = q_true(k,:);
    q_e     = x_hist(k, 1:4);
    q_e_inv = [-q_e(1), -q_e(2), -q_e(3), q_e(4)];
    q_err_w = q_t(4)*q_e_inv(4) - dot(q_t(1:3), q_e_inv(1:3));
    q_err_w = max(min(q_err_w, 1.0), -1.0);
    angle_error_deg(k) = rad2deg(2 * acos(abs(q_err_w)));
end

fprintf('--- 전체 ---\n');
fprintf('RMS  Angle Error: %.4f deg\n', rms(angle_error_deg));
fprintf('Mean Angle Error: %.4f deg\n', mean(angle_error_deg));
gs_idx  = find(mode == 1);
sun_idx = find(mode == 0);
if ~isempty(gs_idx)
    fprintf('--- GS 모드 구간 ---\n');
    fprintf('RMS  Error: %.4f deg\n', rms(angle_error_deg(gs_idx)));
    fprintf('Mean Error: %.4f deg\n', mean(angle_error_deg(gs_idx)));
end
if ~isempty(sun_idx)
    fprintf('--- Sun 모드 구간 ---\n');
    fprintf('RMS  Error: %.4f deg\n', rms(angle_error_deg(sun_idx)));
    fprintf('Mean Error: %.4f deg\n', mean(angle_error_deg(sun_idx)));
end

mode_switch_idx = find(diff(mode) ~= 0);

%% 6. 플롯

% --- (1) 자이로 바이어스 ---
figure('Name','Gyro Bias','Color','w','Position',[100,100,1000,800]);
for i = 1:3
    subplot(3,1,i);
    plot(t_sim, true_bias(:,i), 'k--','LineWidth',1.5,'DisplayName','True Bias'); hold on;
    plot(t_sim, x_hist(:,4+i),  'r',  'LineWidth',1.2,'DisplayName','EKF Estimate');
    grid on;
    ylabel(['\beta_', char(119+i), ' [rad/s]'],'FontSize',12,'FontWeight','bold');
    all_v = [true_bias(:,i); x_hist(:,4+i)];
    lo = min(all_v); hi = max(all_v);
    if abs(hi-lo) > 1e-12, ylim([lo-abs(lo)*0.1, hi+abs(hi)*0.1]); end
    if i==1
        title('Gyro Bias Estimation','FontSize',14,'FontWeight','bold');
        legend('Location','best','FontSize',11);
    end
    if i==3, xlabel('Time [sec]','FontSize',12,'FontWeight','bold'); end
end

% --- (2) 쿼터니언 ---
figure('Name','Quaternion','Color','w','Position',[150,150,1000,800]);
q_names = {'q_x','q_y','q_z','q_w (Scalar)'};
for i = 1:4
    subplot(4,1,i);
    plot(t_sim, q_true(:,i), 'k--','LineWidth',1.5,'DisplayName','True'); hold on;
    plot(t_sim, x_hist(:,i), 'b',  'LineWidth',1.2,'DisplayName','EKF');
    grid on; ylim([-1.1,1.1]);
    ylabel(q_names{i},'FontSize',12,'FontWeight','bold');
    if i==1
        title('Quaternion Estimation (True vs EKF)','FontSize',14,'FontWeight','bold');
        legend('Location','best','FontSize',11);
    end
    if i==4, xlabel('Time [sec]','FontSize',12,'FontWeight','bold'); end
end

% --- (3) 각도 오차 + mode ---
figure('Name','Angle Error','Color','w','Position',[200,200,1000,450]);
yyaxis left;
plot(t_sim, angle_error_deg, 'k','LineWidth',1.2,'DisplayName','Angle Error');
ylabel('Error [deg]','FontSize',12,'FontWeight','bold');
ylim([0, max(10, max(angle_error_deg)*1.1)]);
yyaxis right;
plot(t_sim, mode, 'r','LineWidth',1.0,'DisplayName','Mode');
ylabel('Mode (0=Sun, 1=GS)','FontSize',12,'FontWeight','bold');
ylim([-0.1, 1.5]);
grid on;
xlabel('Time [sec]','FontSize',12,'FontWeight','bold');
title('Angle Error + Mode','FontSize',14,'FontWeight','bold');
legend({'Angle Error','Mode'},'Location','best','FontSize',11);

% --- (4) 모드 전환 구간 확대 ---
if ~isempty(mode_switch_idx)
    t_zoom_start = max(1, mode_switch_idx(1) - 200);
    t_zoom_end   = min(N, mode_switch_idx(end) + 200);

    figure('Name','Mode Switch Zoom','Color','w','Position',[250,250,1000,400]);
    yyaxis left;
    plot(t_sim(t_zoom_start:t_zoom_end), ...
         angle_error_deg(t_zoom_start:t_zoom_end), 'k','LineWidth',1.2);
    ylabel('Error [deg]','FontSize',12);
    yyaxis right;
    plot(t_sim(t_zoom_start:t_zoom_end), ...
         mode(t_zoom_start:t_zoom_end), 'r','LineWidth',1.0);
    ylabel('Mode','FontSize',12);
    for s = 1:length(mode_switch_idx)
        xline(t_sim(mode_switch_idx(s)), '--b', 'LineWidth',1.5);
    end
    grid on;
    xlabel('Time [sec]','FontSize',12);
end

%% 헬퍼 함수
function s = mode_str(m)
    if m == 0, s = 'Sun';
    else,      s = 'GS';
    end
end