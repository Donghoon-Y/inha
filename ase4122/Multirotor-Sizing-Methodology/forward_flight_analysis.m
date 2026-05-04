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
% 적용 이론 - Pollet et al., ICAS 2020:
%   1. 전진비행 힘 평형 (Eq. 3a/3b)
%   2. 비축 입사각에서 CT, CP 수정 (Leng et al. 모델, Eq. 8-9)
%   3. 기체 항력 모델 (Eq. 10-11)
%   4. 최대 전진속도 = 여유추력 곡선과 항력 곡선의 교점
%
% 참조하는 Workspace 변수 (main.m 출력):
%   필수:
%     propList_considered, operatingPoints, consideredNo
%     mass_Total        — 실측 부품 기반 이륙 중량 [g] (W_N 계산에 사용)
%     thrustHover_Est   — 로터당 호버 추력 [g] (mass_Total_Est 기반)
%     RotorNo, SafetyFactor
%     BattCellNo, BattCellVoltage
%     propSpecification — 1안 프롭 사양 (비교 출력용)
%     motorChosen       — 1안 모터 (비교 출력용)
%     time_hover        — 1안 호버 비행시간 (비교 출력용)
%     best_esc          — 1안 ESC (비교 출력용)
%     mass_Motor_Est    — 모터 질량 상한 초기값 [g]
%
% 이 파일이 생성하는 Workspace 변수 (downstream 파일 참조):
%   prop_ff        — 선정 프롭 데이터 (1×6 cell), ground_effect / arm_wake 참조
%   idx_ff_best    — 선정 프롭 인덱스 (propList_considered 기준)
%   V_max_best     — 최대 전진속도 [m/s]
%   V_ff           — 운용 전진속도 (0.95 * V_max_best) [m/s]
%   motorChosen_ff — 2안 선정 모터 (1×12 cell)
%   best_esc_ff    — 2안 선정 ESC
% =========================================================================

%% 사전 확인: 필요 변수 존재 여부
required_vars = {'propList_considered', 'operatingPoints', 'consideredNo', ...
                 'mass_Total', 'thrustHover_Est', 'RotorNo', 'SafetyFactor', ...
                 'BattCellNo', 'BattCellVoltage', 'propSpecification', ...
                 'motorChosen', 'time_hover', 'best_esc', 'mass_Motor_Est'};
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
% W_N: mass_Total(실측 보정값) 기반. thrustHover_Est(mass_Total_Est 기반)와
% 구분하여 사용. 전진비행 힘 평형에는 실측 중량을 써야 정확함.
W_N = mass_Total * 1e-3 * g_acc;   % 기체 중량 [N]

% 기체 항력 계수 — 기본값 1.0 (Pollet 논문 Table 3)
%   유선형 기체: ~0.4~0.6 / 일반 쿼드콥터: ~0.8~1.2
Cd_frame = 1.0;

% S500 프레임 실측 투영 면적
% S500 스펙: 휠베이스 500mm, 암 220×40mm × 4개, 중앙판 170×140mm
%            랜딩기어 파이프 φ16mm × 200mm × 2개
% Stop  (상면 투영): 암 4개 + 중앙판 = ~0.059 m²
% Sfront(정면 투영): 중앙판 정면 + 앞암 2개 + 랜딩기어 = ~0.021 m²
% DJI 스케일링 법칙은 밀폐형 기체 기준이므로 S500 오픈 프레임에는 직접 측정값 사용
Stop   = 0.059;   % m²  (S500 상면 투영)
Sfront = 0.021;   % m²  (S500 정면 투영, 랜딩기어 포함)

fprintf('\n======= [설계 2안] 전진비행 기동성 최적화 =======\n');
fprintf('기체 중량     : %.1f g  (%.3f N)\n', mass_Total, W_N);
fprintf('Cd_frame      : %.2f\n', Cd_frame);
fprintf('Stop / Sfront : %.4f m² / %.4f m²\n', Stop, Sfront);
fprintf('호버 후보 프롭 수 : %d 개\n', consideredNo);

%% 전진비행 CT/CP 모델 - Pollet Eq. 8
% APC 프롭 데이터 회귀 다항식 (β = pitch/diameter 비)
CT_axial = @(beta, Ja) ...
    0.02791 - 0.06543*Ja + 0.11867*beta + 0.27334*beta^2 - 0.28852*beta^3 ...
    + 0.02104*Ja^3 - 0.23504*Ja^2 + 0.18677*beta*Ja^2;

CP_axial = @(beta, Ja) ...
    0.01813 - 0.06218*beta + 0.00343*Ja + 0.35712*beta^2 - 0.23774*beta^3 ...
    + 0.07549*beta*Ja - 0.1235*Ja^2;

%% 비축 입사각 수정 - Leng et al., Pollet Eq. 9
% J0T, J0P : CT=0, CP=0 이 되는 J_axial
% eta_T, eta_P : 입사각 α 에서의 CT/CP 수정 비율
find_J0 = @(beta, coeff_fn) ...
    fzero(@(Ja) coeff_fn(beta, max(Ja,0)), [0.01, 2.0]);

eta_T = @(alpha, J, beta) ...
    1 + ((J * cos(alpha) / (pi * 0.75))^2) / ...
        (2 * (1 - J*sin(alpha) / max(find_J0(beta, CT_axial), 1e-6)));

eta_P = @(alpha, J, beta) ...
    1 + ((J * cos(alpha) / (pi * 0.75))^2) / ...
        (2 * (1 - J*sin(alpha) / max(find_J0(beta, CP_axial), 1e-6)));

%% 각 prop별 최대 전진속도 계산
V_max_all           = nan(consideredNo, 1);
alpha_opt           = nan(consideredNo, 1);
P_ff_all            = nan(consideredNo, 1);
T_ff_all            = nan(consideredNo, 1);
T_surplus_hover_all = nan(consideredNo, 1);
t_lap_all           = nan(consideredNo, 1);
L_course            = 2.0;        % 왕복 코스 길이 [m]

V_scan = 0 : 0.5 : 70;   % 속도 스캔 범위 [m/s]

for ii = 1:consideredNo
    D_m            = propList_considered{ii,3} * IN2M;
    beta           = propList_considered{ii,4} / propList_considered{ii,3};
    speedLimit_rpm = propList_considered{ii,6};

    T_surplus = zeros(size(V_scan));
    P_ff_scan = zeros(size(V_scan));

    % n_lim: RPM 한계 [Hz] — 여유추력과 소비전력 모두 이 기준으로 계산.
    % 플롯 로직과 동일하게 항상 n_lim 기반으로 계산하여 수치와 그래프 일치.
    n_lim = speedLimit_rpm / 60;

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

        % 총 필요 추력
        T_total_N = sqrt(W_N^2 + Df^2);
        T_one_N   = T_total_N / RotorNo;

        % n_lim 기준 Ja, CT, eta 계산 (플롯 로직과 동일)
        J_lim  = V / (n_lim * D_m);
        Ja_lim = max(V * sin(alpha_k) / (n_lim * D_m), 0);

        CT0 = max(CT_axial(beta, Ja_lim), 0);
        if CT0 <= 0
            T_surplus(kk) = -inf;
            continue
        end

        try, etaT = eta_T(alpha_k, J_lim, beta); catch, etaT = 1; end
        etaT = max(min(etaT, 3), 0.1);

        % RPM 한계에서 낼 수 있는 최대 추력
        T_max_N       = CT0 * etaT * rho * n_lim^2 * D_m^4;
        T_surplus(kk) = T_max_N - T_one_N;

        % 소비 전력: 필요 추력을 낼 때의 RPM 기준으로 계산
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
        P_ff_all(ii)  = P_ff_scan(last_valid);
        alpha_opt(ii) = atan2(0.5*Cd_frame*rho*V_max_all(ii)^2 * ...
            (Stop*sin(0.1)+Sfront*cos(0.1)), W_N);
    end

    % 왕복 시간 점수 계산
    % V=0 여유추력 = 기동 가속력의 원천
    %   T_max(V=0) = CT_axial(beta,0) * rho * n_lim^2 * D^4  (입사각 수정 불필요)
    %   a_max = sqrt(T_total_max^2 - W_N^2) / mass
    %   t_one = 2 * sqrt(L_course / a_max)   (삼각형 속도 프로파일)
    %   10회 왕복 = 20 편도 → t_lap = 20 * t_one
    n_lim_ii     = propList_considered{ii,6} / 60;
    D_ii         = propList_considered{ii,3} * IN2M;
    beta_ii      = propList_considered{ii,4} / propList_considered{ii,3};
    CT0_hover_ii = max(CT_axial(beta_ii, 0), 0);

    T_max_total_ii          = CT0_hover_ii * rho * n_lim_ii^2 * D_ii^4 * RotorNo;
    T_surplus_hover_all(ii) = T_max_total_ii - W_N;

    if T_max_total_ii > W_N
        a_max_ii      = sqrt(max(T_max_total_ii^2 - W_N^2, 0)) / (mass_Total*1e-3);
        t_one_ii      = 2 * sqrt(L_course / max(a_max_ii, 1e-6));
        t_lap_all(ii) = 20 * t_one_ii;
    else
        t_lap_all(ii) = inf;
    end
end

%% 결과 출력
fprintf('\n%-22s %6s %6s %6s %10s %14s %12s\n', ...
    'Propeller', 'D(in)', 'pitch', 'beta', 'Vmax(m/s)', 'T_surplus(N)', 't_10lap(s)');
fprintf('%s\n', repmat('-',1,80));
for ii = 1:consideredNo
    beta_i = propList_considered{ii,4} / propList_considered{ii,3};
    fprintf('%-22s %6.1f %6.1f %6.3f %10.2f %14.2f %12.1f\n', ...
        propList_considered{ii,1}, ...
        propList_considered{ii,3}, propList_considered{ii,4}, ...
        beta_i, V_max_all(ii), T_surplus_hover_all(ii), t_lap_all(ii));
end

%% 최적 prop 선정 - 왕복 시간 최소화 (= 여유추력 최대 = 가속도 최대)
valid_mask = V_max_all > 0 & ~isinf(t_lap_all) & ~isnan(t_lap_all);
if ~any(valid_mask)
    error('전진비행 가능한 prop 후보가 없습니다.');
end

t_lap_filtered = t_lap_all;
t_lap_filtered(~valid_mask) = inf;
[t_lap_best, idx_ff_best] = min(t_lap_filtered);

V_max_best = V_max_all(idx_ff_best);
prop_ff    = propList_considered(idx_ff_best, :);
beta_ff    = prop_ff{4} / prop_ff{3};
D_ff       = prop_ff{3} * IN2M;

fprintf('\n[설계 2안] 선정 Prop: %s\n', prop_ff{1});
fprintf('  직경 %.1fin / 피치 %.1fin / beta=%.3f\n', prop_ff{3}, prop_ff{4}, beta_ff);
fprintf('  여유추력 T_surplus : %.2f N\n', T_surplus_hover_all(idx_ff_best));
fprintf('  최대 전진속도     : %.2f m/s (%.1f km/h)\n', V_max_best, V_max_best*3.6);
fprintf('  10회 왕복 추정    : %.1f 초  (L=%.1fm 기준)\n', t_lap_best, L_course);

%% 전진비행 운용점 계산
% V_ff: 최대속도의 95% 에서 운용점 설정 (안전 여유)
V_ff       = V_max_best * 0.95;
% n_lim_ff는 아래 운용점 계산 블록에서 선언

alpha_ff = 0.1;
for iter = 1:30
    Sref     = Stop * sin(alpha_ff) + Sfront * cos(alpha_ff);
    Df       = 0.5 * Cd_frame * rho * V_ff^2 * Sref;
    alpha_ff = atan2(Df, W_N);
end
Sref_ff    = Stop * sin(alpha_ff) + Sfront * cos(alpha_ff);
Df_ff      = 0.5 * Cd_frame * rho * V_ff^2 * Sref_ff;
T_total_ff = sqrt(W_N^2 + Df_ff^2);
T_one_ff   = T_total_ff / RotorNo;

% n_req_ff를 n_lim을 초기값으로 반복 수렴
% V_ff가 클 때 n_hover_hz 기반 Ja로 계산하면 J가 과도하게 커져
% CT가 0 근방으로 떨어지고 n_req_ff가 발산하는 문제를 방지.
n_lim_ff = propList_considered{idx_ff_best,6} / 60;  % RPM 한계 [Hz]
n_req_ff  = n_lim_ff;   % 초기값: RPM 한계

for conv_iter = 1:30
    Ja_ff_iter  = max(V_ff * sin(alpha_ff) / (n_req_ff * D_ff), 0);
    J_ff_iter   = V_ff / (n_req_ff * D_ff);
    CT0_ff      = max(CT_axial(beta_ff, Ja_ff_iter), 0);
    try, etaT_ff = eta_T(alpha_ff, J_ff_iter, beta_ff); catch, etaT_ff = 1; end
    etaT_ff   = max(min(etaT_ff, 3), 0.1);
    CT_ff_val = CT0_ff * etaT_ff;
    if CT_ff_val <= 0, break; end

    n_new = sqrt(T_one_ff / (CT_ff_val * rho * D_ff^4));
    if abs(n_new - n_req_ff) / max(n_req_ff, 1e-6) < 1e-5, break; end
    n_req_ff = n_new;
end

% RPM 한계 초과 시 클램프
n_req_ff = min(n_req_ff, n_lim_ff);
RPM_ff   = n_req_ff * 60;

Ja_ff   = max(V_ff * sin(alpha_ff) / (n_req_ff * D_ff), 0);
J_ff    = V_ff / (n_req_ff * D_ff);
CP0_ff  = CP_axial(beta_ff, Ja_ff);
try, etaP_ff = eta_P(alpha_ff, J_ff, beta_ff); catch, etaP_ff = 1; end
etaP_ff  = max(min(etaP_ff, 3), 0.1);
P_one_ff = max(CP0_ff * etaP_ff * rho * n_req_ff^3 * D_ff^5, 0);

Torque_ff = P_one_ff / (2*pi*max(n_req_ff, 1e-6));

fprintf('\n[전진비행 운용점 (V=%.1f m/s, %.1f%%Vmax)]\n', V_ff, V_ff/V_max_best*100);
fprintf('  기체 경사각 α : %.2f deg\n', rad2deg(alpha_ff));
fprintf('  기체 항력 Df  : %.2f N\n', Df_ff);
fprintf('  로터 1개 추력 : %.1f g  (%.3f N)\n', T_one_ff/g_acc*1000, T_one_ff);
fprintf('  필요 RPM      : %.0f RPM\n', RPM_ff);
fprintf('  로터 1개 소비전력 : %.1f W\n', P_one_ff);
fprintf('  로터 1개 토크     : %.4f Nm\n', Torque_ff);

%% 2안 모터 선정
fprintf('\n[2안 모터 선정 중...]\n');

% 호버 운용점 (main.m에서 계산됨)
speedHover_ff  = operatingPoints{idx_ff_best,1}(1);
torqueHover_ff = operatingPoints{idx_ff_best,1}(3) * SafetyFactor;

% 전진비행 운용점 (위에서 계산)
speedFF_rpm  = RPM_ff;
torqueFF_nm  = Torque_ff * SafetyFactor;

% 호버와 전진비행 중 더 높은 RPM/토크를 설계 상한으로 사용
speedMax_design  = max(speedHover_ff, speedFF_rpm) * SafetyFactor;
torqueMax_design = max(torqueHover_ff, torqueFF_nm);

spec_mass_ff = mass_Motor_Est;

fprintf('  전진비행 RPM    : %.0f RPM  (여유 포함 %.0f RPM)\n', RPM_ff, speedMax_design);
fprintf('  전진비행 Torque : %.4f Nm  (여유 포함 %.4f Nm)\n', Torque_ff, torqueMax_design);
fprintf('  호버 RPM        : %.0f RPM\n', speedHover_ff);
fprintf('  kV 최소 요구값  : %.0f KV  (= speedMax/0.8/Vbatt)\n', ...
    speedMax_design / (0.8 * BattCellNo * BattCellVoltage));
fprintf('  모터 질량 상한  : %.0f g\n', spec_mass_ff);

motorList_ff = load_motorList( ...
    BattCellNo * BattCellVoltage, ...
    speedMax_design, torqueMax_design, ...
    speedHover_ff, torqueHover_ff, ...
    spec_mass_ff);

% 후보 없으면 질량 상한 단계적 완화
if isempty(motorList_ff)
    fprintf('  [질량 상한 완화 시도]\n');
    for mass_relax = [150, 200, 999]
        motorList_ff = load_motorList( ...
            BattCellNo * BattCellVoltage, ...
            speedMax_design, torqueMax_design, ...
            speedHover_ff, torqueHover_ff, ...
            mass_relax);
        if ~isempty(motorList_ff)
            fprintf('  → 질량 상한 %dg 으로 완화 후 후보 %d개 발견\n', ...
                mass_relax, size(motorList_ff,1));
            break;
        end
    end
end

if isempty(motorList_ff)
    fprintf('[오류] 질량 상한 999g 까지 완화해도 조건 만족 모터 없음.\n');
    fprintf('  kV 최소 요구값 %.0f KV 이상인 모터가 목록에 없거나\n', ...
        speedMax_design / (0.8 * BattCellNo * BattCellVoltage));
    fprintf('  배터리 전압(%dS=%.1fV)이 너무 낮을 수 있습니다.\n', ...
        BattCellNo, BattCellNo*BattCellVoltage);
    motorChosen_ff = {};
    best_esc_ff    = {};
else
    % 전진비행 전력 최소 모터 선정
    [~, idx_motor_ff] = min([motorList_ff{:,8}]);
    motorChosen_ff = motorList_ff(idx_motor_ff,:);

    fprintf('\n[설계 2안] 선정 Motor: %s  (%d KV)\n', ...
        motorChosen_ff{2}, round(motorChosen_ff{5}/10)*10);
    fprintf('  허용전류 : %.0f A  /  질량 : %.0f g\n', ...
        motorChosen_ff{3}, motorChosen_ff{4});
    fprintf('  전진비행 소비전력 : %.0f W  /  효율 : %.1f %%\n', ...
        motorChosen_ff{8}, motorChosen_ff{9});
    fprintf('  호버 소비전력     : %.0f W  /  효율 : %.1f %%\n', ...
        motorChosen_ff{11}, motorChosen_ff{12});

    %% 2안 ESC 선정
    esc_List_ff    = load_escList();
    req_current_ff = ceil(motorChosen_ff{7}) * 1.2;

    best_esc_ff  = {};
    min_mass_ff  = inf;
    for i = 1:size(esc_List_ff,1)
        if esc_List_ff{i,3} >= req_current_ff && esc_List_ff{i,4} < min_mass_ff
            min_mass_ff = esc_List_ff{i,4};
            best_esc_ff = esc_List_ff(i,:);
        end
    end

    if isempty(best_esc_ff)
        fprintf('[경고] ESC 리스트에 조건 만족하는 제품이 없습니다. (요구 %.1f A)\n', req_current_ff);
    else
        fprintf('\n[설계 2안] 선정 ESC: %s  (허용 %dA, %.1fg)\n', ...
            best_esc_ff{2}, best_esc_ff{3}, best_esc_ff{4});
    end

    %% 2안 호버 시간 계산 (main.m Peukert 배터리 모델과 동일 로직)
    % 잔류 변수 충돌 방지를 위해 명시적으로 초기화
    clear voltage_hover_ff current_hover_ff capacity_hover_ff
    timeStep_ff          = 1/3600;   % 1초 단위 [h] (main.m과 동일)
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
    time_hover_ff = (0:jj-1) * timeStep_ff;
    hover_min_ff  = round(time_hover_ff(end) * 60);

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
    % 1안 최대속도: V_max_all에서 1안 프롭 인덱스로 직접 조회
    V_max_plan1 = V_max_all(temp_propChosen_pos);
    fprintf('%-22s  %12.1f  %12.1f\n', 'Max speed (m/s)', V_max_plan1, V_max_best);
    fprintf('%s\n', repmat('-', 1, 50));
end

%% 그래프: 전진속도 vs 여유추력 (1안 vs 2안 비교)
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
            Sref  = Stop * sin(alpha_k) + Sfront * cos(alpha_k);
            Df_k  = 0.5 * Cd_frame * rho * V^2 * Sref;
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
    if target_ii == prop1_pos,   lbl = ['1안: ' lbl]; ls = '--'; end
    if target_ii == idx_ff_best, lbl = ['2안: ' lbl]; ls = '-';  end
    plot(V_scan, T_surplus_plot, ls, 'LineWidth', 2, 'DisplayName', lbl);
end

yline(0, 'k:', 'LineWidth', 1.5, 'DisplayName', '여유추력=0 (Vmax 한계)');
xlabel('전진 속도 V [m/s]', 'FontSize', 12);
ylabel('여유 추력 [g]', 'FontSize', 12);
title('[설계 2안] 전진속도 vs 여유추력 (RPM 한계 기준)', 'FontSize', 13);
legend('Location', 'southwest', 'FontSize', 10);
grid on;
xlim([0, max(V_scan)]);

fprintf('\n[완료] forward_flight_analysis.m 실행 완료\n');