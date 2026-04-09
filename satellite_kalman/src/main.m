clc; clear;
thisFile = mfilename('fullpath');
thisDir  = fileparts(thisFile);
addpath(genpath(thisDir));

%% 1. 데이터 로드
fprintf("Data loading...\n");
true_data   = readmatrix("my_satellite_data.csv");
t_sim       = true_data(:, 1);
q_true      = true_data(:, 2:5);   % [qx, qy, qz, qw]
w_true      = true_data(:, 6:8);
sun_eci_ref = true_data(:, 9:11);
r_sat_eci   = true_data(:, 12:14);
r_gs_eci    = true_data(:, 15:17);
mode        = true_data(:, 18);    % 0: Sun모드, 1: GS모드

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

q_quat = 1e-6;
q_bias = rrw^2;
Q = diag([q_quat*ones(1,4), q_bias*ones(1,3)]);

% 측정 노이즈
R_sun   = eye(3) * deg2rad(1.0)^2;   % 태양센서  1 deg
R_nadir = eye(3) * deg2rad(2.0)^2;   % Nadir     2 deg (지구센서는 다소 부정확)
R_gs    = eye(3) * deg2rad(0.5)^2;   % 지상국    0.5 deg

x_hist = zeros(N, 7);
P_hist = zeros(N, 7);

fprintf("EKF Estimation Start...\n");
cnt = 0;

%% 4. EKF Loop
for k = 1:N

    %% --- Prediction ---
    w_curr = w_meas(k,:)' - x_est(5:7);

    Omega = [ 0,          w_curr(3), -w_curr(2),  w_curr(1);
             -w_curr(3),  0,          w_curr(1),  w_curr(2);
              w_curr(2), -w_curr(1),  0,           w_curr(3);
             -w_curr(1), -w_curr(2), -w_curr(3),  0         ];

    theta = norm(w_curr) * dt;
    if theta > 1e-12
        exp_Omega = eye(4)*cos(0.5*theta) + Omega*(sin(0.5*theta)/norm(w_curr));
        x_est(1:4) = exp_Omega * x_est(1:4);
    else
        x_est(1:4) = x_est(1:4) + 0.5 * Omega * x_est(1:4) * dt;
    end
    x_est(1:4) = x_est(1:4) / norm(x_est(1:4));

    q   = x_est(1:4);
    Xi  = [ q(4), -q(3),  q(2);
            q(3),  q(4), -q(1);
           -q(2),  q(1),  q(4);
           -q(1), -q(2), -q(3)];
    F          = eye(7);
    F(1:4,1:4) = eye(4) + 0.5 * Omega * dt;
    F(1:4,5:7) = -0.5 * Xi * dt;

    P = F * P * F' + Q;

    %% --- Correction ---
    C_est  = dcm_q(x_est(1:4));
    C_true = dcm_q(q_true(k,:));

    % Nadir 벡터 (항상 사용 가능)
    v_nadir = -r_sat_eci(k,:)';
    v_nadir = v_nadir / norm(v_nadir);

    if mode(k) == 0
        %% --- Sun 모드: 태양 + Nadir 두 벡터 동시 사용 (6x1) ---
        v_sun = sun_eci_ref(k,:)';
        v_sun = v_sun / norm(v_sun);

        z_meas = [C_true * v_sun   + randn(3,1)*deg2rad(1.0);
                  C_true * v_nadir + randn(3,1)*deg2rad(2.0)];
        z_hat  = [C_est  * v_sun;
                  C_est  * v_nadir];

        H = [jacobian_h_q(x_est(1:4), v_sun),   zeros(3,3);
             jacobian_h_q(x_est(1:4), v_nadir),  zeros(3,3)];  % 6x7

        R_curr = blkdiag(R_sun, R_nadir);  % 6x6

    else
        %% --- GS 모드: 지상국 + Nadir 두 벡터 동시 사용 (6x1) ---
        v_gs = (r_gs_eci(k,:) - r_sat_eci(k,:))';
        v_gs = v_gs / norm(v_gs);

        z_meas = [C_true * v_gs    + randn(3,1)*deg2rad(0.5);
                  C_true * v_nadir + randn(3,1)*deg2rad(2.0)];
        z_hat  = [C_est  * v_gs;
                  C_est  * v_nadir];

        H = [jacobian_h_q(x_est(1:4), v_gs),     zeros(3,3);
             jacobian_h_q(x_est(1:4), v_nadir),  zeros(3,3)];  % 6x7

        R_curr = blkdiag(R_gs, R_nadir);  % 6x6
    end

    % Outlier rejection
    innov = z_meas - z_hat;
    if norm(innov(1:3)) > deg2rad(30) || norm(innov(4:6)) > deg2rad(30)
        x_hist(k,:) = x_est';
        P_hist(k,:) = diag(P)';
        continue;
    end

    % 칼만 게인 (올바른 공식)
    S = H * P * H' + R_curr;
    K = P * H' / S;

    % 상태 업데이트
    x_est      = x_est + K * innov;
    x_est(1:4) = x_est(1:4) / norm(x_est(1:4));

    % P 업데이트 (Joseph form)
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
fprintf('RMS  Angle Error: %.4f deg\n', rms(angle_error_deg));
fprintf('Mean Angle Error: %.4f deg\n', mean(angle_error_deg));

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
        title('Gyro Bias Estimation (Random Walk Model)','FontSize',14,'FontWeight','bold');
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
        title('Quaternion Attitude Estimation (True vs EKF)','FontSize',14,'FontWeight','bold');
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
title('Total Attitude Estimation Angle Error','FontSize',14,'FontWeight','bold');
legend({'Angle Error','Mode'},'Location','best','FontSize',11);