%% =========================================================================
% 설계 2안: 기동성(Maneuverability) 최적화 - 전진비행 해석
%
% [실행 순서] main.m 을 먼저 실행한 뒤 이 파일을 실행하십시오.
%
% [목적]
%   main.m의 호버 후보 prop(propList_considered) 중에서
%   전진비행 최대속도를 최대화하는 prop / motor / ESC 를 선정합니다.
%
% [적용 이론 - Pollet et al., ICAS 2020]
%   1. 전진비행 힘 평형 (Eq. 3a/3b)
%   2. 비축 입사각에서 CT, CP 수정 (Leng et al. 모델, Eq. 8-9)
%   3. 기체 항력 모델 (Eq. 10-11)
%   4. 최대 전진속도 = 여유추력 곡선과 항력 곡선의 교점
%
% =========================================================================

%% ── [사전 확인] main.m 실행 여부 ────────────────────────────────────────
if ~exist('propList_considered','var') || ~exist('mass_Total','var') || ...
   ~exist('thrustHover_Est','var')
    error('main.m 을 먼저 실행하십시오.');
end

%% ── [물리 상수] ──────────────────────────────────────────────────────────
rho    = 1.225;   % 공기밀도 [kg/m³]
g_acc  = 9.81;    % 중력가속도 [m/s²]
IN2M   = 0.0254;  % inch → m

%% ── [기체 파라미터] ──────────────────────────────────────────────────────
% ※ 아래 값은 main.m 에서 가져옵니다. 필요 시 직접 수정하십시오.
W_N     = mass_Total * 1e-3 * g_acc;   % 기체 중량 [N]

% 기체 항력 계수 (Cd) — 기본값 1.0 (Pollet 논문 Table 3)
%   - 잘 정비된 유선형 기체: ~0.4~0.6
%   - 일반 쿼드콥터: ~0.8~1.2
Cd_frame = 1.0;

% ── S500 프레임 실측 면적 직접 지정 ─────────────────────────────────────
% S500 스펙: 휠베이스 500mm, 암 220×40mm × 4개, 중앙판 170×140mm
%            랜딩기어 파이프 φ16mm × 200mm × 2개
%
% Stop  (상면 투영): 암 4개(0.220×0.040×4) + 중앙판(0.170×0.140) = ~0.059 m²
% Sfront(정면 투영): 중앙판 정면 + 앞암 2개 + 랜딩기어 = ~0.021 m²
%
% ※ DJI 스케일링 법칙(M_ref 기준)은 폴딩/밀폐형 기체 기준이라 S500 오픈 프레임에
%    적용 시 Stop이 ~20% 과소평가됨 → 실측값 직접 사용
Stop   = 0.059;   % m²  (S500 상면 투영 실측)
Sfront = 0.021;   % m²  (S500 정면 투영 실측, 랜딩기어 포함)

fprintf('\n======= 전진비행 기동성 최적화 =======\n');
fprintf('기체 중량  : %.1f g  (%.3f N)\n', mass_Total, W_N);
fprintf('Cd_frame   : %.2f\n', Cd_frame);
fprintf('Stop / Sfront : %.4f m² / %.4f m²\n', Stop, Sfront);
fprintf('호버 후보 프롭 수 : %d 개\n', consideredNo);

%% ── [전진비행 CT/CP 모델 - Pollet Eq. 8] ────────────────────────────────
% APC 프롭 데이터 회귀 다항식 (β = pitch/diameter 비)
%   CT_axial = f(β, J_axial)
%   CP_axial = g(β, J_axial)

CT_axial = @(beta, Ja) ...
    0.02791 - 0.06543*Ja + 0.11867*beta + 0.27334*beta^2 - 0.28852*beta^3 ...
    + 0.02104*Ja^3 - 0.23504*Ja^2 + 0.18677*beta*Ja^2;

CP_axial = @(beta, Ja) ...
    0.01813 - 0.06218*beta + 0.00343*Ja + 0.35712*beta^2 - 0.23774*beta^3 ...
    + 0.07549*beta*Ja - 0.1235*Ja^2;

%% ── [비축 입사각 수정 - Leng et al., Pollet Eq. 9] ──────────────────────
% J0T, J0P : CT=0, CP=0 이 되는 J_axial (수치 탐색으로 계산)
% η_T, η_P : 입사각 α 에서의 CT/CP 수정 비율

find_J0 = @(beta, coeff_fn) ...
    fzero(@(Ja) coeff_fn(beta, max(Ja,0)), [0.01, 2.0]);  % J_axial where coeff=0

eta_T = @(alpha, J, beta) ...
    1 + ((J * cos(alpha) / (pi * 0.75))^2) / ...
        (2 * (1 - J*sin(alpha) / max(find_J0(beta, CT_axial), 1e-6)));

eta_P = @(alpha, J, beta) ...
    1 + ((J * cos(alpha) / (pi * 0.75))^2) / ...
        (2 * (1 - J*sin(alpha) / max(find_J0(beta, CP_axial), 1e-6)));

%% ── [각 prop별 최대 전진속도 계산] ─────────────────────────────────────
V_max_all  = nan(consideredNo, 1);   % 최대 전진속도 [m/s]
alpha_opt  = nan(consideredNo, 1);   % 최대속도에서 기체 경사각 [rad]
P_ff_all   = nan(consideredNo, 1);   % 전진비행에서 소비 전력 [W] (1개 로터)
T_ff_all   = nan(consideredNo, 1);   % 전진비행에서 1개 로터 추력 [N]

V_scan = 0 : 0.5 : 40;   % 속도 스캔 범위 [m/s]

for ii = 1:consideredNo
    D_m    = propList_considered{ii,3} * IN2M;   % 직경 [m]
    beta   = propList_considered{ii,4} / propList_considered{ii,3}; % pitch/diam
    n_rpm  = operatingPoints{ii,1}(1);           % 호버 RPM
    n_hz   = n_rpm / 60;                         % [Hz]
    speedLimit_rpm = propList_considered{ii,6};  % 제조사 RPM 한계

    % 전진속도 V에서의 여유추력 계산
    T_surplus = zeros(size(V_scan));
    P_ff_scan = zeros(size(V_scan));

    for kk = 1:length(V_scan)
        V = V_scan(kk);

        % ── 기체 경사각 α 수치 해석 (힘 평형, Pollet Eq. 4) ──────────────
        % 수평: N_rotors*T*sin(α) = Df*cos(θFP)   (θFP=0, 수평비행)
        % 수직: N_rotors*T*cos(α) = W + Df*sin(θFP) ≈ W (수평비행 시)
        % → tan(α) = Df / W
        % Df = 0.5 * Cd * rho * V^2 * Sref,  Sref = Stop*sin(α) + Sfront*cos(α)
        % 반복 수렴

        alpha_k = 0.1;   % 초기값 [rad]
        for iter = 1:20
            Sref  = Stop * sin(alpha_k) + Sfront * cos(alpha_k);
            Df    = 0.5 * Cd_frame * rho * V^2 * Sref;
            alpha_new = atan2(Df, W_N);
            if abs(alpha_new - alpha_k) < 1e-5, break; end
            alpha_k = alpha_new;
        end
        alpha_k = max(alpha_k, 0);

        % 총 필요 추력 (Pollet Eq. 5, θFP=0)
        T_total_N = sqrt(W_N^2 + Df^2);   % [N]
        T_one_N   = T_total_N / RotorNo;  % 로터 1개당 필요 추력 [N]

        % ── 전진비행 CT (입사각 수정) ──────────────────────────────────────
        J      = V / (n_hz * D_m);         % 전진비 (full inflow)
        Ja     = V * sin(alpha_k) / (n_hz * D_m); % 축방향 전진비
        Ja     = max(Ja, 0);

        CT0 = CT_axial(beta, Ja);
        if CT0 <= 0
            T_surplus(kk) = -inf;
            continue
        end

        try
            etaT = eta_T(alpha_k, J, beta);
        catch
            etaT = 1;
        end
        etaT = max(min(etaT, 3), 0.1);   % 물리적 범위 제한

        % RPM을 높여서 필요 추력 달성 가능한지 확인 ──────────────────────────
        % T = CT * rho * n^2 * D^4  →  n = sqrt(T / (CT * rho * D^4))
        CT_ff = CT0 * etaT;
        if CT_ff <= 0
            T_surplus(kk) = -inf;
            continue
        end

        n_req = sqrt(T_one_N / (CT_ff * rho * D_m^4));  % [Hz]

        % RPM 한계 초과 여부
        if n_req * 60 > speedLimit_rpm
            T_surplus(kk) = -inf;
            continue
        end

        % 여유 추력 = (RPM 한계에서 발생 가능한 최대 추력) - (필요 추력)
        n_lim    = speedLimit_rpm / 60;
        CT0_lim  = CT_axial(beta, V * sin(alpha_k) / (n_lim * D_m));
        CT0_lim  = max(CT0_lim, 0);
        try, etaT_lim = eta_T(alpha_k, J, beta); catch, etaT_lim = 1; end
        etaT_lim  = max(min(etaT_lim, 3), 0.1);
        T_max_N   = CT0_lim * etaT_lim * rho * n_lim^2 * D_m^4;  % 1개 로터 최대 추력

        T_surplus(kk) = T_max_N - T_one_N;   % [N]

        % 전진비행 소비 전력 (CP 수정)
        CP0 = CP_axial(beta, Ja);
        try, etaP = eta_P(alpha_k, J, beta); catch, etaP = 1; end
        etaP = max(min(etaP, 3), 0.1);
        CP_ff = CP0 * etaP;
        P_ff_scan(kk) = max(CP_ff * rho * n_req^3 * D_m^5, 0);   % [W/rotor]
    end % V_scan

    % 최대 전진속도 = T_surplus가 0이 되는 점 (가장 큰 V)
    idx_valid = find(T_surplus > 0);
    if isempty(idx_valid)
        V_max_all(ii) = 0;
    else
        last_valid = idx_valid(end);
        if last_valid < length(V_scan)
            % 선형 보간으로 정밀화
            V1 = V_scan(last_valid);
            V2 = V_scan(last_valid + 1);
            T1 = T_surplus(last_valid);
            T2 = T_surplus(last_valid + 1);
            if T2 < T1
                V_max_all(ii) = V1 + (V2 - V1) * T1 / (T1 - T2);
            else
                V_max_all(ii) = V_scan(last_valid);
            end
        else
            V_max_all(ii) = V_scan(last_valid);
        end
        % 최대속도에서 전력 기록
        P_ff_all(ii) = P_ff_scan(last_valid);
        alpha_opt(ii) = atan2(0.5 * Cd_frame * rho * V_max_all(ii)^2 * ...
            (Stop * sin(0.1) + Sfront * cos(0.1)), W_N);
    end
end % prop loop

%% ── [결과 출력] ─────────────────────────────────────────────────────────
fprintf('\n%-22s %8s %6s %6s %10s\n', 'Propeller', 'D(in)', 'pitch', 'beta', 'Vmax(m/s)');
fprintf('%s\n', repmat('-',1,60));
for ii = 1:consideredNo
    beta_i = propList_considered{ii,4} / propList_considered{ii,3};
    fprintf('%-22s %8.1f %6.1f %6.3f %10.2f\n', ...
        propList_considered{ii,1}, ...
        propList_considered{ii,3}, ...
        propList_considered{ii,4}, ...
        beta_i, V_max_all(ii));
end

%% ── [최적 prop 선정 - 최대속도 최대화] ──────────────────────────────────
[V_max_best, idx_ff_best] = max(V_max_all);

if isnan(V_max_best) || V_max_best == 0
    error('전진비행 가능한 prop 후보가 없습니다. 파라미터를 확인하십시오.');
end

prop_ff = propList_considered(idx_ff_best, :);
beta_ff = prop_ff{4} / prop_ff{3};
D_ff    = prop_ff{3} * IN2M;

fprintf('\n 선정 Prop: %s\n', prop_ff{1});
fprintf('  직경 %.1fin / 피치 %.1fin / β=%.3f\n', prop_ff{3}, prop_ff{4}, beta_ff);
fprintf('  예측 최대 전진속도: %.2f m/s (%.1f km/h)\n', V_max_best, V_max_best*3.6);

%% ── [전진비행 운용점 계산] ──────────────────────────────────────────────
% 최대속도에서의 α, 필요 추력, RPM 계산
V_ff   = V_max_best * 0.95;   % 안전 여유 5% 아래에서 운용점 설정
n_hover_hz = operatingPoints{idx_ff_best,1}(1) / 60;

alpha_ff = 0.1;
for iter = 1:30
    Sref  = Stop * sin(alpha_ff) + Sfront * cos(alpha_ff);
    Df    = 0.5 * Cd_frame * rho * V_ff^2 * Sref;
    alpha_ff = atan2(Df, W_N);
end
Sref_ff   = Stop * sin(alpha_ff) + Sfront * cos(alpha_ff);
Df_ff     = 0.5 * Cd_frame * rho * V_ff^2 * Sref_ff;
T_total_ff = sqrt(W_N^2 + Df_ff^2);
T_one_ff   = T_total_ff / RotorNo;   % [N], 로터 1개

Ja_ff  = V_ff * sin(alpha_ff) / (n_hover_hz * D_ff);
CT0_ff = CT_axial(beta_ff, max(Ja_ff, 0));
try, etaT_ff = eta_T(alpha_ff, V_ff/(n_hover_hz*D_ff), beta_ff); catch, etaT_ff = 1; end
etaT_ff = max(min(etaT_ff, 3), 0.1);
CT_ff_val = CT0_ff * etaT_ff;

% 전진비행에서 필요 RPM
n_req_ff  = sqrt(T_one_ff / max(CT_ff_val * rho * D_ff^4, 1e-10));   % [Hz]
RPM_ff    = n_req_ff * 60;

% 전진비행 소비 전력
CP0_ff = CP_axial(beta_ff, max(Ja_ff, 0));
try, etaP_ff = eta_P(alpha_ff, V_ff/(n_hover_hz*D_ff), beta_ff); catch, etaP_ff = 1; end
etaP_ff = max(min(etaP_ff, 3), 0.1);
P_one_ff = CP0_ff * etaP_ff * rho * n_req_ff^3 * D_ff^5;   % [W]

% 토크
Torque_ff = P_one_ff / (2*pi*n_req_ff);   % [Nm]

fprintf('\n[전진비행 운용점 (V=%.1f m/s, %.1f%%Vmax)]\n', V_ff, V_ff/V_max_best*100);
fprintf('  기체 경사각 α : %.2f deg\n', rad2deg(alpha_ff));
fprintf('  기체 항력 Df  : %.2f N\n', Df_ff);
fprintf('  로터 1개 추력 : %.1f g  (%.3f N)\n', T_one_ff/g_acc*1000, T_one_ff);
fprintf('  필요 RPM      : %.0f RPM\n', RPM_ff);
fprintf('  로터 1개 소비전력 : %.1f W\n', P_one_ff);
fprintf('  로터 1개 토크     : %.4f Nm\n', Torque_ff);

%% ── [2안 모터 선정] ─────────────────────────────────────────────────────
% 전진비행 운용점을 기준으로 모터 필터링
fprintf('\n[모터 선정 중...]\n');

% ── 운용점 정리 ──────────────────────────────────────────────────────────
% 호버 운용점 (main.m에서 이미 계산됨)
speedHover_ff  = operatingPoints{idx_ff_best,1}(1);   % [RPM]
torqueHover_ff = operatingPoints{idx_ff_best,1}(3) * SafetyFactor; % [Nm]

% 전진비행 운용점 (이 파일에서 계산)
speedFF_rpm   = RPM_ff;                        % [RPM]
torqueFF_nm   = Torque_ff * SafetyFactor;      % [Nm]

% load_motorList 는 "prop_speedMax" 로 kV 적합성을 판단함
% (조건: 0.8 * Vbatt * kV > prop_speedMax)
% → 호버와 전진비행 중 더 높은 RPM을 상한으로 사용
speedMax_design = max(speedHover_ff, speedFF_rpm) * SafetyFactor; % [RPM]
torqueMax_design = max(torqueHover_ff, torqueFF_nm);               % [Nm]

% 질량 상한: main.m 의 mass_Motor_Est 를 기준으로 하되,
%           실제 모터 목록에 해당 질량 이하가 충분히 있는지 확인 후 자동 완화
spec_mass_ff = mass_Motor_Est;

fprintf('  전진비행 RPM     : %.0f RPM  (여유 포함 %.0f RPM)\n', RPM_ff, speedMax_design);
fprintf('  전진비행 Torque  : %.4f Nm  (여유 포함 %.4f Nm)\n', Torque_ff, torqueMax_design);
fprintf('  호버 RPM         : %.0f RPM\n', speedHover_ff);
fprintf('  kV 최소 요구값   : %.0f KV  (= speedMax/0.8/Vbatt)\n', ...
    speedMax_design / (0.8 * BattCellNo * BattCellVoltage));
fprintf('  모터 질량 상한   : %.0f g\n', spec_mass_ff);

motorList_ff = load_motorList( ...
    BattCellNo * BattCellVoltage, ...    % 배터리 전압
    speedMax_design, torqueMax_design, ...% 최대 운용점 (호버/전진 중 큰 값)
    speedHover_ff, torqueHover_ff, ...   % 호버 운용점
    spec_mass_ff);                       % 모터 질량 상한

% 후보가 없으면 질량 상한을 단계적으로 완화
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
    fprintf('  배터리 전압(%dS=%.1fV)이 너무 낮을 수 있습니다.\n', BattCellNo, BattCellNo*BattCellVoltage);
else
    % 전진비행 전력 최소 모터 선정
    [~, idx_motor_ff] = min([motorList_ff{:,8}]);
    motorChosen_ff = motorList_ff(idx_motor_ff,:);

    fprintf('\n 선정 Motor: %s  (%d KV)\n', ...
        motorChosen_ff{2}, round(motorChosen_ff{5}/10)*10);
    fprintf('  허용전류 : %.0f A  /  질량 : %.0f g\n', motorChosen_ff{3}, motorChosen_ff{4});
    fprintf('  전진비행 소비전력 : %.0f W  /  효율 : %.1f %%\n', ...
        motorChosen_ff{8}, motorChosen_ff{9});
    fprintf('  호버 소비전력     : %.0f W  /  효율 : %.1f %%\n', ...
        motorChosen_ff{11}, motorChosen_ff{12});

    %% ── [2안 ESC 선정] ───────────────────────────────────────────────────
    esc_List_ff = load_escList();
    req_current_ff = ceil(motorChosen_ff{7}) * 1.2;   % 전진비행 최대 전류 * 여유 20%

    best_esc_ff = {}; min_mass_ff = inf;
    for i = 1:size(esc_List_ff,1)
        if esc_List_ff{i,3} >= req_current_ff && esc_List_ff{i,4} < min_mass_ff
            min_mass_ff = esc_List_ff{i,4};
            best_esc_ff = esc_List_ff(i,:);
        end
    end

    if isempty(best_esc_ff)
        fprintf('[경고] ESC 리스트에 조건 만족하는 제품이 없습니다. (요구 %.1f A)\n', req_current_ff);
    else
        fprintf('\n 선정 ESC: %s  (허용 %dA, %.1fg)\n', ...
            best_esc_ff{2}, best_esc_ff{3}, best_esc_ff{4});
    end

    %% ── [설계 1안 vs 2안 비교 요약] ─────────────────────────────────────
    fprintf('\n\n========== 설계안 비교 요약 ==========\n');
    fprintf('%-22s  %12s  %12s\n', '', '1안(Endurance)', '2안(Maneuver)');
    fprintf('%s\n', repmat('-', 1, 50));
    fprintf('%-22s  %12s  %12s\n', 'Prop', ...
        propSpecification{1}, prop_ff{1});
    fprintf('%-22s  %12.1f  %12.1f\n', 'Prop D (in)', ...
        propSpecification{2}, prop_ff{3});
    fprintf('%-22s  %12.1f  %12.1f\n', 'Prop pitch (in)', ...
        propSpecification{3}, prop_ff{4});
    fprintf('%-22s  %12s  %12s\n', 'Motor', ...
        motorChosen{2}, motorChosen_ff{2});
    fprintf('%-22s  %12d  %12d\n', 'Motor kV', ...
        round(motorChosen{5}/10)*10, round(motorChosen_ff{5}/10)*10);
    fprintf('%-22s  %12.0f  %12.0f\n', 'Hover P/rotor (W)', ...
        motorChosen{11}, motorChosen_ff{11});
    if ~isempty(best_esc_ff)
        fprintf('%-22s  %12s  %12s\n', 'ESC', ...
            best_esc{2}, best_esc_ff{2});
    end
    fprintf('%-22s  %12.0f  %12.0f\n', 'Hover time (min)', ...
        round(time_hover(end)*60), NaN);
    fprintf('%-22s  %12s  %12.1f\n', 'Max speed (m/s)', ...
        '(hover only)', V_max_best);
    fprintf('%s\n', repmat('-', 1, 50));
end

%% ── [그래프: 전진속도 vs 여유추력] ─────────────────────────────────────
% 선정된 prop과 1안 prop을 비교 시각화
prop1_pos = temp_propChosen_pos;   % 1안 prop 위치

figure('Name', '전진비행 여유추력 곡선', 'Position', [100 100 900 550]);
hold on;
colors = lines(3);

% 1안 prop
for target_ii = unique([prop1_pos, idx_ff_best])
    D_t   = propList_considered{target_ii,3} * IN2M;
    beta_t = propList_considered{target_ii,4} / propList_considered{target_ii,3};
    n_hz_t = operatingPoints{target_ii,1}(1) / 60;
    n_lim_t = propList_considered{target_ii,6} / 60;

    T_surplus_plot = zeros(size(V_scan));
    Drag_plot      = zeros(size(V_scan));

    for kk = 1:length(V_scan)
        V = V_scan(kk);
        alpha_k = 0.05;
        for iter = 1:15
            Sref  = Stop * sin(alpha_k) + Sfront * cos(alpha_k);
            Df_k  = 0.5 * Cd_frame * rho * V^2 * Sref;
            alpha_k = atan2(Df_k, W_N);
        end
        T_req_N = sqrt(W_N^2 + (0.5*Cd_frame*rho*V^2*(Stop*sin(alpha_k)+Sfront*cos(alpha_k)))^2) / RotorNo;

        Ja_k = max(V * sin(alpha_k) / (n_lim_t * D_t), 0);
        CT0  = max(CT_axial(beta_t, Ja_k), 0);
        try, etaT_k = eta_T(alpha_k, V/(n_lim_t*D_t), beta_t); catch, etaT_k=1; end
        etaT_k = max(min(etaT_k,3),0.1);
        T_max_plot = CT0 * etaT_k * rho * n_lim_t^2 * D_t^4;

        T_surplus_plot(kk) = (T_max_plot - T_req_N) * 1000 / g_acc;   % → g
        Drag_plot(kk)      = T_req_N * 1000 / g_acc;
    end

    lbl = propList_considered{target_ii,1};
    if target_ii == prop1_pos,    lbl = ['1안: ' lbl]; ls = '--'; end
    if target_ii == idx_ff_best,  lbl = ['2안: ' lbl]; ls = '-'; end
    plot(V_scan, T_surplus_plot, ls, 'LineWidth', 2, 'DisplayName', lbl);
end

yline(0, 'k:', 'LineWidth', 1.5, 'DisplayName', '여유추력=0 (Vmax 한계)');
xlabel('전진 속도 V [m/s]', 'FontSize', 12);
ylabel('여유 추력 [g]', 'FontSize', 12);
title('전진속도 vs 여유추력 (RPM 한계 기준)', 'FontSize', 13);
legend('Location', 'southwest', 'FontSize', 10);
grid on;
xlim([0, max(V_scan)]);

fprintf('\n[완료] forward_flight_analysis.m 실행 완료\n');