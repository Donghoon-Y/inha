%% =========================================================================
% 설계 2안: 기동성(Maneuverability) 최적화 - 전진비행 해석
%
% 실행 순서:
%   main.m 실행 후 이 파일 실행
%   이후: ground_effect_analysis.m → arm_wake_analysis.m
%
% 목적:
%   main.m의 호버 후보 prop(propList_considered) 중에서
%   전진비행 최대속도를 최대화하는 prop / motor / ESC 를 선정합니다.
%
% [수정] 질량 수렴 루프 추가:
%   2안에서 선정된 motor/ESC/prop 무게가 1안과 다를 수 있으므로
%   mass_Total_ff 를 반복 갱신하여 W_N(기체 중량)을 수렴시킴.
%   수렴 완료 후 downstream 해석(ground_effect, arm_wake)을 위해
%   mass_Total_ff 를 workspace 에 저장함.
%
% 적용 이론 - Pollet et al., ICAS 2020:
%   1. 전진비행 힘 평형 (Eq. 3a/3b)
%   2. 비축 입사각에서 CT, CP 수정 (Leng et al. 모델, Eq. 8-9)
%   3. 기체 항력 모델 (Eq. 10-11)
%   4. 최대 전진속도 = 여유추력 곡선과 항력 곡선의 교점
%
% 참조하는 Workspace 변수 (main.m 출력):
%   필수:
%     propList_considered, operatingPoints, consideredNo
%     mass_Total        — 실측 부품 기반 이륙 중량 [g]
%     mass_Motor_Est    — 모터 질량 상한 초기값 [g]
%     thrustHover_Est   — 로터당 호버 추력 [g]
%     RotorNo, SafetyFactor
%     BattCellNo, BattCellVoltage
%     BattCapacity, BattPeukertConstant, BattHourRating, BattVoltageSagConstant
%     propSpecification — 1안 프롭 사양 (비교 출력용)
%     motorChosen       — 1안 모터 (비교 출력용)
%     time_hover        — 1안 호버 비행시간 (비교 출력용)
%     best_esc          — 1안 ESC (비교 출력용)
%     temp_propChosen_pos — propList_considered 내 1안 프롭 인덱스
%
% 이 파일이 생성하는 Workspace 변수 (downstream 파일 참조):
%   prop_ff        — 선정 프롭 데이터 (1×6 cell)
%   idx_ff_best    — 선정 프롭 인덱스 (propList_considered 기준)
%   V_max_best     — 최대 전진속도 [m/s]
%   V_ff           — 운용 전진속도 (0.95 * V_max_best) [m/s]
%   motorChosen_ff — 2안 선정 모터 (1×12 cell)
%   best_esc_ff    — 2안 선정 ESC
%   mass_Total_ff  — 2안 수렴 후 최종 기체 질량 [g]  ★ NEW
% =========================================================================

%% 사전 확인: 필요 변수 존재 여부
required_vars = {'propList_considered', 'operatingPoints', 'consideredNo', ...
                 'mass_Total', 'thrustHover_Est', 'RotorNo', 'SafetyFactor', ...
                 'BattCellNo', 'BattCellVoltage', 'propSpecification', ...
                 'motorChosen', 'time_hover', 'best_esc', 'mass_Motor_Est', ...
                 'BattCapacity', 'BattPeukertConstant', 'BattHourRating', ...
                 'BattVoltageSagConstant', 'temp_propChosen_pos'};
missing = {};
for v = required_vars
    if ~exist(v{1}, 'var'), missing{end+1} = v{1}; end
end
if ~isempty(missing)
    error('forward_flight_analysis: 다음 변수가 없습니다. main.m을 먼저 실행하세요.\n  누락: %s', ...
        strjoin(missing, ', '));
end

%% 물리 상수
rho   = 1.225;   % 공기밀도 [kg/m³]
g_acc = 9.81;    % 중력가속도 [m/s²]
IN2M  = 0.0254;  % inch → m

%% 기체 파라미터
Cd_frame = 1.0;   % 기체 항력 계수 (S500 오픈 프레임 기준)
Stop   = 0.059;   % m²  (S500 상면 투영)
Sfront = 0.021;   % m²  (S500 정면 투영, 랜딩기어 포함)

fprintf('\n======= [설계 2안] 전진비행 기동성 최적화 =======\n');
fprintf('Cd_frame      : %.2f\n', Cd_frame);
fprintf('Stop / Sfront : %.4f m² / %.4f m²\n', Stop, Sfront);
fprintf('호버 후보 프롭 수 : %d 개\n', consideredNo);

%% CT/CP 모델 함수 정의 (Pollet Eq. 8 - APC 회귀 다항식)
CT_axial = @(beta, Ja) ...
    0.02791 - 0.06543*Ja + 0.11867*beta + 0.27334*beta^2 - 0.28852*beta^3 ...
    + 0.02104*Ja^3 - 0.23504*Ja^2 + 0.18677*beta*Ja^2;

CP_axial = @(beta, Ja) ...
    0.01813 - 0.06218*beta + 0.00343*Ja + 0.35712*beta^2 - 0.23774*beta^3 ...
    + 0.07549*beta*Ja - 0.1235*Ja^2;

%% 비축 입사각 수정 함수 (Leng et al., Pollet Eq. 9)
find_J0 = @(beta, coeff_fn) ...
    fzero(@(Ja) coeff_fn(beta, max(Ja,0)), [0.01, 2.0]);

eta_T = @(alpha, J, beta) ...
    1 + ((J * cos(alpha) / (pi * 0.75))^2) / ...
        (2 * (1 - J*sin(alpha) / max(find_J0(beta, CT_axial), 1e-6)));

eta_P = @(alpha, J, beta) ...
    1 + ((J * cos(alpha) / (pi * 0.75))^2) / ...
        (2 * (1 - J*sin(alpha) / max(find_J0(beta, CP_axial), 1e-6)));

%% =========================================================================
%  질량 수렴 루프 (OUTER LOOP)
%
%  수렴 변수: mass_Total_ff [g]
%  - 초기값: mass_Total (1안 실측 기반)
%  - 매 iter: 선정된 2안 motor/ESC 무게로 mass_Total_ff 갱신
%  - 종료 조건: |Δmass| < 1g  또는 최대 10회
%
%  수렴 구조:
%    iter 1: W_N = mass_Total 기반 → prop/motor/ESC 선정
%    iter 2: W_N = (mass_Total - 1안부품 + 2안부품) 기반 → 재선정
%    ...반복...
%    수렴 시 mass_Total_ff 확정
% =========================================================================

MAX_OUTER_ITER = 10;
CONV_TOL_G     = 1.0;   % 수렴 허용 오차 [g]
V_scan         = 0 : 0.5 : 70;   % 속도 스캔 범위 [m/s]
L_course       = 2.0;             % 왕복 코스 길이 [m]

% 1안 부품 무게 추출 (수렴 루프에서 질량 차이 계산에 사용)
mass_motor_plan1 = motorChosen{4};   % 1안 모터 1개 질량 [g]
mass_esc_plan1   = best_esc{4};      % 1안 ESC 1개 질량 [g]

% prop 무게: propList_considered{ii,5} 에 있으면 사용, 없으면 0으로 처리
% (propList 컬럼 구조에 따라 인덱스 확인 필요)
has_prop_mass = (size(propList_considered, 2) >= 7);
if has_prop_mass
    mass_prop_plan1 = propSpecification{6};   % 1안 프롭 1개 질량 [g] (컬럼6 가정)
else
    mass_prop_plan1 = 0;
end

% 초기 질량 설정
mass_Total_ff = mass_Total;

% 수렴 루프 외부에서 선언 (수렴 실패 시에도 변수 존재 보장)
motorChosen_ff = {};
best_esc_ff    = {};
idx_ff_best    = 1;
V_max_best     = 0;
prop_ff        = propList_considered(1,:);

fprintf('\n--- 질량 수렴 루프 시작 ---\n');
fprintf('%-6s  %10s  %10s  %10s\n', 'Iter', 'mass_ff[g]', 'Δmass[g]', '선정Prop');

for outer_iter = 1:MAX_OUTER_ITER

    % ---------------------------------------------------------------
    % 현재 질량 기반 W_N 갱신
    % ---------------------------------------------------------------
    W_N = mass_Total_ff * 1e-3 * g_acc;   % 기체 중량 [N]

    % ---------------------------------------------------------------
    % 각 prop별 최대 전진속도 및 기동 성능 계산
    % ---------------------------------------------------------------
    V_max_all           = nan(consideredNo, 1);
    P_ff_all            = nan(consideredNo, 1);
    T_surplus_hover_all = nan(consideredNo, 1);
    t_lap_all           = nan(consideredNo, 1);

    for ii = 1:consideredNo
        D_m            = propList_considered{ii,3} * IN2M;
        beta           = propList_considered{ii,4} / propList_considered{ii,3};
        speedLimit_rpm = propList_considered{ii,6};
        n_lim          = speedLimit_rpm / 60;   % [Hz]

        T_surplus = zeros(size(V_scan));
        P_ff_scan = zeros(size(V_scan));

        for kk = 1:length(V_scan)
            V = V_scan(kk);

            % 기체 경사각 α 수치 해석 (힘 평형, Pollet Eq. 4)
            alpha_k = 0.1;
            for iter = 1:20
                Sref      = Stop * sin(alpha_k) + Sfront * cos(alpha_k);
                Df        = 0.5 * Cd_frame * rho * V^2 * Sref;
                alpha_new = atan2(Df, W_N);
                if abs(alpha_new - alpha_k) < 1e-5, break; end
                alpha_k = alpha_new;
            end
            alpha_k = max(alpha_k, 0);

            % 총 필요 추력 및 로터 1개 추력
            T_total_N = sqrt(W_N^2 + Df^2);
            T_one_N   = T_total_N / RotorNo;

            % n_lim 기준 Ja, CT, eta 계산
            J_lim  = V / (n_lim * D_m);
            Ja_lim = max(V * sin(alpha_k) / (n_lim * D_m), 0);

            CT0 = max(CT_axial(beta, Ja_lim), 0);
            if CT0 <= 0
                T_surplus(kk) = -inf;
                continue
            end

            try, etaT = eta_T(alpha_k, J_lim, beta); catch, etaT = 1; end
            etaT = max(min(etaT, 3), 0.1);

            T_max_N       = CT0 * etaT * rho * n_lim^2 * D_m^4;
            T_surplus(kk) = T_max_N - T_one_N;

            % 소비 전력 (필요 추력 기준 RPM)
            CT_ff = CT0 * etaT;
            if CT_ff > 0 && T_one_N > 0
                n_req = sqrt(T_one_N / (CT_ff * rho * D_m^4));
                CP0   = CP_axial(beta, max(V * sin(alpha_k) / (n_req * D_m), 0));
                try, etaP = eta_P(alpha_k, V/(n_req*D_m), beta); catch, etaP = 1; end
                etaP = max(min(etaP, 3), 0.1);
                P_ff_scan(kk) = max(CP0 * etaP * rho * n_req^3 * D_m^5, 0);
            end
        end

        % 최대 전진속도 계산
        idx_valid = find(T_surplus > 0);
        if isempty(idx_valid)
            V_max_all(ii) = 0;
        else
            last_valid = idx_valid(end);
            if last_valid < length(V_scan)
                V1 = V_scan(last_valid);   V2 = V_scan(last_valid+1);
                T1 = T_surplus(last_valid); T2 = T_surplus(last_valid+1);
                if T2 < T1
                    V_max_all(ii) = V1 + (V2-V1)*T1/(T1-T2);
                else
                    V_max_all(ii) = V_scan(last_valid);
                end
            else
                V_max_all(ii) = V_scan(last_valid);
            end
            P_ff_all(ii) = P_ff_scan(last_valid);
        end

        % 왕복 시간 점수 계산 (삼각형 속도 프로파일 가정)
        n_lim_ii     = propList_considered{ii,6} / 60;
        D_ii         = propList_considered{ii,3} * IN2M;
        beta_ii      = propList_considered{ii,4} / propList_considered{ii,3};
        CT0_hover_ii = max(CT_axial(beta_ii, 0), 0);

        T_max_total_ii          = CT0_hover_ii * rho * n_lim_ii^2 * D_ii^4 * RotorNo;
        T_surplus_hover_all(ii) = T_max_total_ii - W_N;

        if T_max_total_ii > W_N
            a_max_ii      = sqrt(max(T_max_total_ii^2 - W_N^2, 0)) / (mass_Total_ff*1e-3);
            t_one_ii      = 2 * sqrt(L_course / max(a_max_ii, 1e-6));
            t_lap_all(ii) = 20 * t_one_ii;
        else
            t_lap_all(ii) = inf;
        end
    end

    % ---------------------------------------------------------------
    % 최적 prop 선정 (왕복 시간 최소화)
    % ---------------------------------------------------------------
    valid_mask = V_max_all > 0 & ~isinf(t_lap_all) & ~isnan(t_lap_all);
    if ~any(valid_mask)
        error('전진비행 가능한 prop 후보가 없습니다. (outer_iter=%d)', outer_iter);
    end

    t_lap_filtered = t_lap_all;
    t_lap_filtered(~valid_mask) = inf;
    [t_lap_best, idx_ff_best_cur] = min(t_lap_filtered);

    V_max_best_cur = V_max_all(idx_ff_best_cur);
    prop_ff_cur    = propList_considered(idx_ff_best_cur, :);
    beta_ff_cur    = prop_ff_cur{4} / prop_ff_cur{3};
    D_ff_cur       = prop_ff_cur{3} * IN2M;

    % ---------------------------------------------------------------
    % 전진비행 운용점 계산 (V_ff = 0.95 * V_max)
    % ---------------------------------------------------------------
    V_ff_cur = V_max_best_cur * 0.95;

    alpha_ff = 0.1;
    for iter = 1:30
        Sref     = Stop * sin(alpha_ff) + Sfront * cos(alpha_ff);
        Df_ff    = 0.5 * Cd_frame * rho * V_ff_cur^2 * Sref;
        alpha_ff = atan2(Df_ff, W_N);
    end
    Sref_ff    = Stop * sin(alpha_ff) + Sfront * cos(alpha_ff);
    Df_ff      = 0.5 * Cd_frame * rho * V_ff_cur^2 * Sref_ff;
    T_total_ff = sqrt(W_N^2 + Df_ff^2);
    T_one_ff   = T_total_ff / RotorNo;

    % RPM 수렴 (n_lim 초기값)
    n_lim_ff = propList_considered{idx_ff_best_cur,6} / 60;
    n_req_ff  = n_lim_ff;

    for conv_iter = 1:30
        Ja_ff_iter = max(V_ff_cur * sin(alpha_ff) / (n_req_ff * D_ff_cur), 0);
        J_ff_iter  = V_ff_cur / (n_req_ff * D_ff_cur);
        CT0_ff     = max(CT_axial(beta_ff_cur, Ja_ff_iter), 0);
        try, etaT_ff = eta_T(alpha_ff, J_ff_iter, beta_ff_cur); catch, etaT_ff = 1; end
        etaT_ff   = max(min(etaT_ff, 3), 0.1);
        CT_ff_val = CT0_ff * etaT_ff;
        if CT_ff_val <= 0, break; end

        n_new = sqrt(T_one_ff / (CT_ff_val * rho * D_ff_cur^4));
        if abs(n_new - n_req_ff) / max(n_req_ff, 1e-6) < 1e-5, break; end
        n_req_ff = n_new;
    end
    n_req_ff = min(n_req_ff, n_lim_ff);
    RPM_ff   = n_req_ff * 60;

    Ja_ff   = max(V_ff_cur * sin(alpha_ff) / (n_req_ff * D_ff_cur), 0);
    J_ff    = V_ff_cur / (n_req_ff * D_ff_cur);
    CP0_ff  = CP_axial(beta_ff_cur, Ja_ff);
    try, etaP_ff = eta_P(alpha_ff, J_ff, beta_ff_cur); catch, etaP_ff = 1; end
    etaP_ff  = max(min(etaP_ff, 3), 0.1);
    P_one_ff = max(CP0_ff * etaP_ff * rho * n_req_ff^3 * D_ff_cur^5, 0);
    Torque_ff = P_one_ff / (2*pi*max(n_req_ff, 1e-6));

    % ---------------------------------------------------------------
    % 2안 모터 선정
    % ---------------------------------------------------------------
    speedHover_ff  = operatingPoints{idx_ff_best_cur,1}(1);
    torqueHover_ff = operatingPoints{idx_ff_best_cur,1}(3) * SafetyFactor;

    speedMax_design  = max(speedHover_ff, RPM_ff) * SafetyFactor;
    torqueMax_design = max(torqueHover_ff, Torque_ff * SafetyFactor);
    spec_mass_ff     = mass_Motor_Est;

    motorList_ff = load_motorList( ...
        BattCellNo * BattCellVoltage, ...
        speedMax_design, torqueMax_design, ...
        speedHover_ff, torqueHover_ff, ...
        spec_mass_ff);

    % 질량 상한 단계적 완화
    if isempty(motorList_ff)
        for mass_relax = [150, 200, 999]
            motorList_ff = load_motorList( ...
                BattCellNo * BattCellVoltage, ...
                speedMax_design, torqueMax_design, ...
                speedHover_ff, torqueHover_ff, ...
                mass_relax);
            if ~isempty(motorList_ff), break; end
        end
    end

    if isempty(motorList_ff)
        warning('outer_iter %d: 조건 만족 모터 없음. 이전 결과 유지.', outer_iter);
        break;
    end

    % 전진비행 전력 최소 모터 선정
    [~, idx_motor_ff] = min([motorList_ff{:,8}]);
    motorChosen_ff_cur = motorList_ff(idx_motor_ff,:);

    % ---------------------------------------------------------------
    % 2안 ESC 선정
    % ---------------------------------------------------------------
    esc_List_ff    = load_escList();
    req_current_ff = ceil(motorChosen_ff_cur{7}) * 1.2;

    best_esc_ff_cur = {};
    min_mass_esc    = inf;
    for i = 1:size(esc_List_ff,1)
        if esc_List_ff{i,3} >= req_current_ff && esc_List_ff{i,4} < min_mass_esc
            min_mass_esc    = esc_List_ff{i,4};
            best_esc_ff_cur = esc_List_ff(i,:);
        end
    end

    % ---------------------------------------------------------------
    % prop 무게 (있으면 반영)
    % ---------------------------------------------------------------
    if has_prop_mass
        mass_prop_ff_cur  = prop_ff_cur{6};
        mass_prop_plan1_v = mass_prop_plan1;
    else
        mass_prop_ff_cur  = 0;
        mass_prop_plan1_v = 0;
    end

    % ---------------------------------------------------------------
    % 2안 기준 mass_Total 갱신
    %   = mass_Total
    %     - (1안 모터 × RotorNo) + (2안 모터 × RotorNo)
    %     - (1안 ESC  × RotorNo) + (2안 ESC  × RotorNo)
    %     - (1안 prop × RotorNo) + (2안 prop × RotorNo)
    % ---------------------------------------------------------------
    mass_motor_ff = motorChosen_ff_cur{4};   % 2안 모터 1개 [g]

    if ~isempty(best_esc_ff_cur)
        mass_esc_ff = best_esc_ff_cur{4};    % 2안 ESC 1개 [g]
    else
        mass_esc_ff = mass_esc_plan1;        % ESC 선정 실패 시 1안 값 유지
    end

    mass_Total_ff_new = mass_Total ...
        + (mass_motor_ff   - mass_motor_plan1) * RotorNo ...
        + (mass_esc_ff     - mass_esc_plan1  ) * RotorNo ...
        + (mass_prop_ff_cur - mass_prop_plan1_v) * RotorNo;

    delta_mass = abs(mass_Total_ff_new - mass_Total_ff);

    fprintf('%-6d  %10.1f  %10.2f  %s\n', ...
        outer_iter, mass_Total_ff_new, delta_mass, prop_ff_cur{1});

    % 결과 저장 (수렴 여부와 무관하게 최신 결과 유지)
    mass_Total_ff  = mass_Total_ff_new;
    motorChosen_ff = motorChosen_ff_cur;
    best_esc_ff    = best_esc_ff_cur;
    idx_ff_best    = idx_ff_best_cur;
    V_max_best     = V_max_best_cur;
    prop_ff        = prop_ff_cur;
    V_ff           = V_ff_cur;

    % 수렴 판정
    if delta_mass < CONV_TOL_G
        fprintf('→ 수렴 완료 (iter=%d, Δmass=%.3fg)\n', outer_iter, delta_mass);
        break;
    end

    if outer_iter == MAX_OUTER_ITER
        fprintf('[경고] 최대 반복(%d회) 도달. 마지막 결과 사용.\n', MAX_OUTER_ITER);
    end
end

fprintf('--- 질량 수렴 루프 종료 ---\n');
fprintf('최종 2안 기체 질량 : %.1f g  (1안 대비 %+.1f g)\n', ...
    mass_Total_ff, mass_Total_ff - mass_Total);

%% 최종 prop/운용점 재계산 (수렴된 W_N 기준)
W_N      = mass_Total_ff * 1e-3 * g_acc;
beta_ff  = prop_ff{4} / prop_ff{3};
D_ff     = prop_ff{3} * IN2M;
n_lim_ff = prop_ff{6} / 60;

% 최종 전진비행 운용점
alpha_ff = 0.1;
for iter = 1:30
    Sref     = Stop * sin(alpha_ff) + Sfront * cos(alpha_ff);
    Df_ff    = 0.5 * Cd_frame * rho * V_ff^2 * Sref;
    alpha_ff = atan2(Df_ff, W_N);
end
Sref_ff    = Stop * sin(alpha_ff) + Sfront * cos(alpha_ff);
Df_ff      = 0.5 * Cd_frame * rho * V_ff^2 * Sref_ff;
T_total_ff = sqrt(W_N^2 + Df_ff^2);
T_one_ff   = T_total_ff / RotorNo;

n_req_ff = n_lim_ff;
for conv_iter = 1:30
    Ja_ff_iter = max(V_ff * sin(alpha_ff) / (n_req_ff * D_ff), 0);
    J_ff_iter  = V_ff / (n_req_ff * D_ff);
    CT0_ff     = max(CT_axial(beta_ff, Ja_ff_iter), 0);
    try, etaT_ff = eta_T(alpha_ff, J_ff_iter, beta_ff); catch, etaT_ff = 1; end
    etaT_ff   = max(min(etaT_ff, 3), 0.1);
    CT_ff_val = CT0_ff * etaT_ff;
    if CT_ff_val <= 0, break; end
    n_new = sqrt(T_one_ff / (CT_ff_val * rho * D_ff^4));
    if abs(n_new - n_req_ff) / max(n_req_ff, 1e-6) < 1e-5, break; end
    n_req_ff = n_new;
end
n_req_ff  = min(n_req_ff, n_lim_ff);
RPM_ff    = n_req_ff * 60;

Ja_ff    = max(V_ff * sin(alpha_ff) / (n_req_ff * D_ff), 0);
J_ff     = V_ff / (n_req_ff * D_ff);
CP0_ff   = CP_axial(beta_ff, Ja_ff);
try, etaP_ff = eta_P(alpha_ff, J_ff, beta_ff); catch, etaP_ff = 1; end
etaP_ff  = max(min(etaP_ff, 3), 0.1);
P_one_ff = max(CP0_ff * etaP_ff * rho * n_req_ff^3 * D_ff^5, 0);
Torque_ff = P_one_ff / (2*pi*max(n_req_ff, 1e-6));

%% 결과 출력
fprintf('\n======= 최종 선정 결과 =======\n');
fprintf('[설계 2안] 선정 Prop: %s\n', prop_ff{1});
fprintf('  직경 %.1fin / 피치 %.1fin / beta=%.3f\n', prop_ff{3}, prop_ff{4}, beta_ff);
fprintf('  여유추력 T_surplus : %.2f N\n', T_surplus_hover_all(idx_ff_best));
fprintf('  최대 전진속도     : %.2f m/s (%.1f km/h)\n', V_max_best, V_max_best*3.6);
fprintf('  10회 왕복 추정    : %.1f 초  (L=%.1fm 기준)\n', t_lap_best, L_course);

fprintf('\n[전진비행 운용점 (V=%.1f m/s, %.1f%%Vmax)]\n', V_ff, V_ff/V_max_best*100);
fprintf('  기체 중량 W_N     : %.3f N  (mass_Total_ff = %.1f g)\n', W_N, mass_Total_ff);
fprintf('  기체 경사각 α     : %.2f deg\n', rad2deg(alpha_ff));
fprintf('  기체 항력 Df      : %.2f N\n', Df_ff);
fprintf('  로터 1개 추력     : %.1f g  (%.3f N)\n', T_one_ff/g_acc*1000, T_one_ff);
fprintf('  필요 RPM          : %.0f RPM\n', RPM_ff);
fprintf('  로터 1개 소비전력 : %.1f W\n', P_one_ff);
fprintf('  로터 1개 토크     : %.4f Nm\n', Torque_ff);

if ~isempty(motorChosen_ff)
    fprintf('\n[설계 2안] 선정 Motor: %s  (%d KV)\n', ...
        motorChosen_ff{2}, round(motorChosen_ff{5}/10)*10);
    fprintf('  허용전류 : %.0f A  /  질량 : %.0f g\n', ...
        motorChosen_ff{3}, motorChosen_ff{4});
    fprintf('  전진비행 소비전력 : %.0f W  /  효율 : %.1f %%\n', ...
        motorChosen_ff{8}, motorChosen_ff{9});
    fprintf('  호버 소비전력     : %.0f W  /  효율 : %.1f %%\n', ...
        motorChosen_ff{11}, motorChosen_ff{12});
end

if ~isempty(best_esc_ff)
    fprintf('\n[설계 2안] 선정 ESC: %s  (허용 %dA, %.1fg)\n', ...
        best_esc_ff{2}, best_esc_ff{3}, best_esc_ff{4});
end

%% 2안 호버 시간 계산 (Peukert 배터리 모델, main.m과 동일 로직)
if ~isempty(motorChosen_ff)
    clear voltage_hover_ff current_hover_ff capacity_hover_ff
    timeStep_ff          = 1/3600;
    BattCapacity_Ah_ff   = BattCapacity / 1000;
    voltage_hover_ff(1)  = BattCellNo * (BattCellVoltage + 0.5);
    current_hover_ff(1)  = motorChosen_ff{11} * RotorNo / voltage_hover_ff(1);
    capacity_hover_ff(1) = (current_hover_ff(1)^(1-BattPeukertConstant)) * ...
        (BattHourRating^(1-BattPeukertConstant)) * (BattCapacity_Ah_ff^BattPeukertConstant);

    jj = 1;
    while voltage_hover_ff(jj) > BattCellVoltage*BattCellNo && jj*timeStep_ff < 2
        voltage_hover_ff(jj+1) = voltage_hover_ff(1) - ...
            (BattVoltageSagConstant/capacity_hover_ff(1)) * ...
            (capacity_hover_ff(1) - capacity_hover_ff(jj));
        current_hover_ff(jj+1) = motorChosen_ff{11} * RotorNo / voltage_hover_ff(jj+1);
        capacity_hover_ff(jj+1) = (current_hover_ff(jj+1)^(1-BattPeukertConstant)) * ...
            (BattHourRating^(1-BattPeukertConstant)) * (BattCapacity_Ah_ff^BattPeukertConstant) ...
            - sum(current_hover_ff(2:end) * timeStep_ff);
        jj = jj + 1;
    end
    time_hover_ff_arr = (0:jj-1) * timeStep_ff;
    hover_min_ff      = round(time_hover_ff_arr(end) * 60);

    %% 설계 1안 vs 2안 비교 요약
    fprintf('\n\n========== 설계안 비교 요약 ==========\n');
    fprintf('%-22s  %12s  %12s\n', '', '1안(Endurance)', '2안(Maneuver)');
    fprintf('%s\n', repmat('-', 1, 50));
    fprintf('%-22s  %12s  %12s\n', 'Prop', propSpecification{1}, prop_ff{1});
    fprintf('%-22s  %12.1f  %12.1f\n', 'Prop D (in)', propSpecification{2}, prop_ff{3});
    fprintf('%-22s  %12.1f  %12.1f\n', 'Prop pitch (in)', propSpecification{3}, prop_ff{4});
    fprintf('%-22s  %12s  %12s\n', 'Motor', motorChosen{2}, motorChosen_ff{2});
    fprintf('%-22s  %12d  %12d\n', 'Motor kV', ...
        round(motorChosen{5}/10)*10, round(motorChosen_ff{5}/10)*10);
    fprintf('%-22s  %12.0f  %12.0f\n', 'Hover P/rotor (W)', ...
        motorChosen{11}, motorChosen_ff{11});
    if ~isempty(best_esc) && ~isempty(best_esc_ff)
        fprintf('%-22s  %12s  %12s\n', 'ESC', best_esc{2}, best_esc_ff{2});
    end
    fprintf('%-22s  %12.0f  %12.0f\n', 'Hover time (min)', ...
        round(time_hover(end)*60), hover_min_ff);
    % 1안 기체 질량 vs 2안 수렴 후 기체 질량
    fprintf('%-22s  %12.0f  %12.0f\n', 'mass_Total (g)', ...
        mass_Total, mass_Total_ff);
    % 1안 최대속도 조회
    V_max_plan1 = V_max_all(temp_propChosen_pos);
    fprintf('%-22s  %12.1f  %12.1f\n', 'Max speed (m/s)', V_max_plan1, V_max_best);
    fprintf('%s\n', repmat('-', 1, 50));
end

%% 그래프: 전진속도 vs 여유추력 (1안 vs 2안)
prop1_pos = temp_propChosen_pos;

figure('Name', '[설계 2안] 전진비행 여유추력 곡선', 'Position', [100 100 900 550]);
hold on;

for target_ii = unique([prop1_pos, idx_ff_best])
    D_t     = propList_considered{target_ii,3} * IN2M;
    beta_t  = propList_considered{target_ii,4} / propList_considered{target_ii,3};
    n_lim_t = propList_considered{target_ii,6} / 60;

    T_surplus_plot = zeros(size(V_scan));

    for kk = 1:length(V_scan)
        V = V_scan(kk);
        alpha_k = 0.05;
        for iter = 1:15
            Sref    = Stop * sin(alpha_k) + Sfront * cos(alpha_k);
            Df_k    = 0.5 * Cd_frame * rho * V^2 * Sref;
            alpha_k = atan2(Df_k, W_N);
        end
        T_req_N = sqrt(W_N^2 + ...
            (0.5*Cd_frame*rho*V^2*(Stop*sin(alpha_k)+Sfront*cos(alpha_k)))^2) / RotorNo;

        Ja_k = max(V * sin(alpha_k) / (n_lim_t * D_t), 0);
        CT0  = max(CT_axial(beta_t, Ja_k), 0);
        try, etaT_k = eta_T(alpha_k, V/(n_lim_t*D_t), beta_t); catch, etaT_k=1; end
        etaT_k = max(min(etaT_k,3),0.1);
        T_max_plot = CT0 * etaT_k * rho * n_lim_t^2 * D_t^4;

        T_surplus_plot(kk) = (T_max_plot - T_req_N) * 1000 / g_acc;
    end

    lbl = propList_considered{target_ii,1};
    ls  = '-';
    if target_ii == prop1_pos,   lbl = ['1안: ' lbl]; ls = '--'; end
    if target_ii == idx_ff_best, lbl = ['2안: ' lbl]; ls = '-';  end
    plot(V_scan, T_surplus_plot, ls, 'LineWidth', 2, 'DisplayName', lbl);
end

yline(0, 'k:', 'LineWidth', 1.5, 'DisplayName', '여유추력=0 (Vmax 한계)');
xlabel('전진 속도 V [m/s]', 'FontSize', 12);
ylabel('여유 추력 [g]', 'FontSize', 12);
title(sprintf('[설계 2안] 전진속도 vs 여유추력 (수렴 mass=%.0fg)', mass_Total_ff), 'FontSize', 13);
legend('Location', 'southwest', 'FontSize', 10);
grid on;
xlim([0, max(V_scan)]);

fprintf('\n[완료] forward_flight_analysis.m 실행 완료\n');
fprintf('  → mass_Total_ff = %.1f g 가 workspace에 저장됨\n', mass_Total_ff);
fprintf('  → ground_effect_analysis.m / arm_wake_analysis.m 에서 이 값을 사용하세요.\n');