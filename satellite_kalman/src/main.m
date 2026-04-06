clc; clear;
thisFile = mfilename('fullpath');
thisDir  = fileparts(thisFile);
addpath(genpath(thisDir)); 

%% 1. 데이터 로드
fprintf("Data loading...\n");
true_data = readmatrix("my_satellite_data.csv");
t_sim       = true_data(:, 1);
q_true      = true_data(:, 2:5);   % [q1, q2, q3, q4] -> [qx, qy, qz, qw]
w_true      = true_data(:, 6:8);   
sun_eci_ref = true_data(:, 9:11);  
r_sat_eci   = true_data(:, 12:14); 
r_gs_eci    = true_data(:, 15:17); 
mode        = true_data(:, 18);    

N = length(t_sim);
dt = t_sim(2) - t_sim(1);

%% 2. 센서 노이즈 및 랜덤 워크 바이어스 생성
arw = 0.001;      
rrw = 1e-5;       
true_bias = zeros(N,3);
true_bias(1, :) = [0.01, -0.01, 0.005];
for k = 2:N
    true_bias(k, :) = true_bias(k-1, :) + randn(1,3)*rrw*sqrt(dt);
end
w_meas = w_true + true_bias + randn(N, 3) * arw;

%% 3. EKF 초기화
x_est = [q_true(1,:)'; 0; 0; 0]; 
P = diag([1e-6*ones(1,4), 1e-5*ones(1,3)]); 

% Tuning Parameters
q_quat = 1e-9; 
q_bias = rrw^2 ; % 바이어스 변화 허용치
Q = diag([q_quat*ones(1,4), q_bias*ones(1,3)]);

R_std = deg2rad(1.0); % 측정 오차를 여유 있게 설정하여 수렴 유도
R_curr_mat = eye(3) * R_std^2;

x_hist = zeros(N, 7);
P_hist = zeros(N, 7);

fprintf("EKF Estimation Start (User DCM Applied).......\n");
cnt = 0; 

%% 4. EKF Loop
for k = 1:N
    % --- Step 1: Prediction ---
    w_curr = w_meas(k, :)' - x_est(5:7);
    
    % Omega Matrix (JPL: q_dot = 0.5 * Omega * q)
    Omega = [ 0,      w_curr(3), -w_curr(2), w_curr(1);
             -w_curr(3), 0,       w_curr(1), w_curr(2);
              w_curr(2), -w_curr(1), 0,      w_curr(3);
             -w_curr(1), -w_curr(2), -w_curr(3), 0];
    
    % Quaternion Prediction (Exponential Map)
    theta = norm(w_curr) * dt;
    if theta > 1e-12
        exp_Omega = eye(4)*cos(0.5*theta) + Omega*(sin(0.5*theta)/norm(w_curr));
        x_est(1:4) = exp_Omega * x_est(1:4);
    else
        x_est(1:4) = x_est(1:4) + 0.5 * Omega * x_est(1:4) * dt;
    end
    x_est(1:4) = x_est(1:4) / norm(x_est(1:4));
    
    % F matrix (State Transition)
    F = eye(7);
    F(1:4, 1:4) = eye(4) + 0.5 * Omega * dt;
    
    % Xi matrix for Bias Jacobian (df/db)
    q = x_est(1:4);
    Xi = [ q(4), -q(3),  q(2);
           q(3),  q(4), -q(1);
          -q(2),  q(1),  q(4);
          -q(1), -q(2), -q(3)];
    F(1:4, 5:7) = -0.5 * Xi * dt; 
    
    P = F * P * F' + Q;

    % --- Step 2: Correction ---
    if mode(k) == 1 % Ground Mode
        v_ref = (r_gs_eci(k,:) - r_sat_eci(k,:))';
        v_ref = v_ref / norm(v_ref);
    else % Sun Mode
        v_ref = sun_eci_ref(k, :)';
    end
    
    % 측정치 생성 (사용자 dcm_q 사용)
    z_meas = dcm_q(q_true(k,:)) * v_ref + randn(3,1) * deg2rad(0.5);
    
    % 추정치 계산 (사용자 dcm_q 사용)
    z_hat = dcm_q(x_est(1:4)) * v_ref;
    
    % Jacobian H (dz/dq) - 사용자 dcm_q 미분 형태
    H = zeros(3, 7);
    H(:, 1:4) = jacobian_h_q(x_est(1:4), v_ref);
    
    % Update
    K = P * H' / (H * H' + R_curr_mat);
    x_est = x_est + K * (z_meas - z_hat);
    x_est(1:4) = x_est(1:4) / norm(x_est(1:4));
    P = (eye(7) - K * H) * P;
    
    % 저장
    x_hist(k, :) = x_est';
    P_hist(k, :) = diag(P)';

    if mod(k, 500) == 0
        fprintf('%d ', k);
        cnt = cnt + 1;
        if mod(cnt, 5) == 0, fprintf('\n'); end
    end
end
fprintf('\nEKF Calculation Complete.\n');

% --- (1) 자이로 바이어스 추정 결과 (기존 그래프 개선) ---
figure('Name', 'EKF Bias Estimation Performance', 'Color', 'w', 'Position', [100, 100, 1000, 800]);
for i = 1:3
    subplot(3,1,i);
    % 참값 (Random Walk) - 검은색 점선
    plot(t_sim, true_bias(:,i), 'k--', 'LineWidth', 1.5, 'DisplayName', 'True Bias'); hold on;
    % EKF 추정값 - 빨간색 실선
    plot(t_sim, x_hist(:,4+i), 'r', 'LineWidth', 1.2, 'DisplayName', 'EKF Estimate');
    
    grid on;
    ylabel(['\beta_', char(119+i), ' [rad/s]'], 'FontSize', 12, 'FontWeight', 'bold');
    
    % y축 범위를 자동으로 맞추되, 초기 튐 현상을 고려하여 적절히 설정
    b_min = min(true_bias(:,i)); b_max = max(true_bias(:,i));
    e_min = min(x_hist(:,4+i)); e_max = max(x_hist(:,4+i));
    y_lim_min = min(b_min, e_min); y_lim_max = max(b_max, e_max);
    ylim([y_lim_min*1.1, y_lim_max*1.1]);
    
    if i==1
        title('Gyro Bias Estimation (Random Walk Model)', 'FontSize', 14, 'FontWeight', 'bold');
        legend('show', 'Location', 'best', 'FontSize', 11);
    end
    if i==3
        xlabel('Time [sec]', 'FontSize', 12, 'FontWeight', 'bold');
    end
end

% --- (2) 쿼터니언 자세 추정 비교 (True vs EKF) ---
figure('Name', 'Quaternion Attitude Estimation', 'Color', 'w', 'Position', [150, 150, 1000, 800]);
q_names = {'q_x', 'q_y', 'q_z', 'q_w (Scalar)'};
for i = 1:4
    subplot(4,1,i);
    % 실제 쿼터니언 - 검은색 점선
    plot(t_sim, q_true(:,i), 'k--', 'LineWidth', 1.5, 'DisplayName', 'True'); hold on;
    % EKF 추정 쿼터니언 - 파란색 실선
    plot(t_sim, x_hist(:,i), 'b', 'LineWidth', 1.2, 'DisplayName', 'EKF');
    
    grid on;
    ylabel(q_names{i}, 'FontSize', 12, 'FontWeight', 'bold');
    ylim([-1.1, 1.1]); % 쿼터니언 성분은 -1~1 사이
    
    if i==1
        title('Quaternion Attitude Estimation Performance (True vs EKF)', 'FontSize', 14, 'FontWeight', 'bold');
        legend('show', 'Location', 'best', 'FontSize', 11);
    end
    if i==4
        xlabel('Time [sec]', 'FontSize', 12, 'FontWeight', 'bold');
    end
end

% --- (3) 전체 자세 오차 (Angle Error in Degrees) ---
% 두 쿼터니언 사이의 각도 오차 계산
angle_error_deg = zeros(N, 1);
for k = 1:N
    % q_err = q_true * inv(q_est) -> 오차 쿼터니언 계산 (JPL 기준)
    q_t = q_true(k, :);
    q_e = x_hist(k, 1:4);
    
    % JPL 표기법 쿼터니언 곱셈 (inv(q) = [-qx, -qy, -qz, qw])
    q_e_inv = [-q_e(1), -q_e(2), -q_e(3), q_e(4)];
    
    % 오차 쿼터니언 q_err = q_t * q_e_inv
    q_err_w = q_t(4)*q_e_inv(4) - dot(q_t(1:3), q_e_inv(1:3));
    
    % 각도 오차 = 2 * acos(|q_err_w|)
    q_err_w = max(min(q_err_w, 1.0), -1.0); % 수치적 안정성 확보
    angle_error_deg(k) = rad2deg(2 * acos(abs(q_err_w)));
end

figure('Name', 'Total Attitude Angle Error', 'Color', 'w', 'Position', [200, 200, 800, 400]);
plot(t_sim, angle_error_deg, 'k', 'LineWidth', 1.2);
grid on;
xlabel('Time [sec]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Error [deg]', 'FontSize', 12, 'FontWeight', 'bold');
title('Total Attitude Estimation Angle Error', 'FontSize', 14, 'FontWeight', 'bold');
y_max_error = max(angle_error_deg);
ylim([0, max(5, y_max_error * 1.1)]); % 오차가 작더라도 최소 5도는 보이도록 설정