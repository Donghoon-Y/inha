function control_warmstart()
%% control_warmstart.m
% Conceptual sizing result -> 6-DOF hover linearization -> PX4 PID warm-start
%
% 본 스크립트는 선행된 개념설계(main.m 등) 결과를 바탕으로
% 드론의 6자유도 운동방정식을 구성하고, 호버링(Hover) 상태에서의 선형화 모델을 통해
% PX4 자동비행장치(QGroundControl)에 입력할 초기 PID 게인(Warm-start params)을 도출합니다.
%
% ── 실행 순서 ──────────────────────────────────────────────────────────────
%   [1] Workspace 변수 추출 (개념설계 결과: Design 1, 2 자동 임포트)
%   [2] 기체/로터 파라미터 빌드 (모터 위치, 회전 방향, Allocation Matrix 구성)
%   [3] APC 프롭 성능 데이터 기반 Hover 트림(RPM, 추력) 도출 및 kT, kQ 국소 피팅
%   [4] 6-DOF 수치적 선형화 (Numerical Linearization -> A, B 상태공간 행렬 추출)
%   [5] 설계 목표 동특성(wn, zeta)을 만족하는 물리적 비율 제어기(Physical Rate PID) 설계
%   [6] 모터 최대 추력 한계점 기반의 축별 제어 여유도(Control Margin & Mscale) 평가
%   [7] 물리적 PID 게인을 PX4 정규화(-1 ~ 1) 게인으로 변환 (안전계수 적용)
%
% ── Workspace에서 가져오는 주요 변수 (Upstream Inputs) ──────────────────────
%   공통 / Design 1 (호버/체공시간 최적화):
%     final_aum_p       [g]   — 최종 기체 총 중량 (프레임+배터리+모터 등 실측 추정)
%     best_arm_len_p    [mm]  — 최적 암 길이 (로터 중심까지의 거리)
%     final_height_m    [m]   — 최종 로터 높이 (CG 대비 Z축 오프셋)
%     propSpecification       — 선정 프롭 사양 {이름, 직경, 피치 등}
%     motorChosen             — 선정 모터 사양 {이름, kV, RPM_max 등}
%     best_esc                — 선정 ESC 정보
%     BattCellNo, BattCellVoltage — 배터리 전압 사양
%
%   Design 2 (기동성 최적화):
%     prop_ff, motorChosen_ff, best_esc_ff
%     (forward_flight_analysis.m 에서 도출된 _ff 접미사 변수들)
%
% ── 수동 입력 필요 변수 (Manual Inputs) ────────────────────────────────────
%   CATIA 관성모멘트 (MOI) - '2-1. CATIA MOI 입력부' 참조:
%     design(1).Ixx, Iyy, Izz [kg*m^2]
%     design(2).Ixx, Iyy, Izz [kg*m^2]
%     ※ 이 값이 NaN이면 물리 모델 구성이 불가하므로 6-DOF 및 PID 계산을 건너뜁니다.
%
% ── 주요 내부 연산 및 출력 변수 (Downstream Outputs) ──────────────────────
%   P (Vehicle Params) :
%     J               [kg*m^2] — 관성모멘트 텐서 행렬
%     Aalloc, Ainv             — 제어 할당 행렬 (Thrust -> Virtual Control) 및 역행렬
%     kT, kQ                   — 호버 RPM 근처에서 2차 곡선 피팅된 추력/토크 계수
%     gamma                    — 요(Yaw) 반작용 토크 계수 (kQ / kT)
%
%   trim / 6-DOF :
%     T_hover_each_N  [N]      — 모터 1개당 요구되는 호버 기준 추력
%     omega_hover     [rad/s]  — 호버링 시 로터 회전 각속도
%     A, B                     — 호버 상태 기준 선형화된 시스템 매트릭스
%
%   PID & Margin :
%     pidPhys                  — 물리 단위(Nm / (rad/s))로 계산된 축별 Rate 게인
%     margin.Mscale_* — 각 축(Roll, Pitch, Yaw)별 물리적 최대 발생 가능 모멘트
%     px4                      — QGC에 직접 입력할 최종 PX4 정규화 파라미터 구조체
%                                (MC_ROLLRATE_P, MC_ROLL_P 등)
% Rotor layout:
%
%             Front +x
%
%          M3(CW)      M1(CCW)
%
%              \        /
%                  CG
%              /        \
%
%          M2(CCW)     M4(CW)
%
% Body frame:
% x: forward
% y: right
% z: down
%
% Rotor direction:
% M1, M2 = CCW
% M3, M4 = CW

clc; close all;

%% ================================================================
% 0. Path setting
% ================================================================
repoRoot = fileparts(mfilename('fullpath'));
addpath(genpath(repoRoot));
cd(repoRoot);

%% ================================================================
% 1. Global configuration
% ================================================================
cfg.g = 9.80665;                 % [m/s^2]
cfg.rho = 1.225;                 % [kg/m^3]
cfg.RotorNo = 4;
cfg.RPM2RAD = 2*pi/60;
cfg.RAD2RPM = 60/(2*pi);

% Local fitting range around hover RPM
cfg.fitBand = 0.20;              % hover RPM ±20%

% Motor/ESC/prop first-order response time constant
cfg.default_tau_m = 0.05;        % [s]

% No-load RPM estimate
cfg.eta_noload = 0.85;

% Rate-loop target dynamics
cfg.zeta_rate = 0.80;
cfg.wn_roll  = 8.0;              % [rad/s]
cfg.wn_pitch = 8.0;              % [rad/s]
cfg.wn_yaw   = 4.0;              % [rad/s]

% Third pole for rate PID design with motor lag
cfg.thirdPoleMultiplier = 5.0;

% PX4 conversion safety factor
cfg.px4SafetyFactor = 0.50;

% PX4 attitude P warm-start
cfg.MC_ROLL_P  = 4.0;
cfg.MC_PITCH_P = 4.0;
cfg.MC_YAW_P   = 2.0;

% Optional: print A, B matrices
cfg.printStateSpace = false;

%% ================================================================
% 2. Design input 생성 (Base Workspace에서 추출)
% ================================================================
design = fetchDesignFromWorkspace(cfg);

%% ================================================================
% 2-1. CATIA MOI 입력부
% ================================================================
% 단위: kg*m^2
% 아래 NaN을 실제 CATIA 계산값으로 교체해야 6-DOF/PID 계산 진행됨

% Design 1
design(1).Ixx = 1;
design(1).Iyy = 1;
design(1).Izz = 1;
design(1).Ixy = 0;
design(1).Ixz = 0;
design(1).Iyz = 0;

% Design 2
design(2).Ixx = 1;
design(2).Iyy = 1;
design(2).Izz = 1;
design(2).Ixy = 0;
design(2).Ixz = 0;
design(2).Iyz = 0;

%% ================================================================
% 3. Load existing APC propeller database
% ================================================================
propList = load_propList();

%% ================================================================
% 4. Main calculation
% ================================================================
results = struct([]);

for k = 1:numel(design)

    fprintf('\n============================================================\n');
    fprintf('%s\n', design(k).name);
    fprintf('============================================================\n');

    P = buildVehicleParams(design(k), propList, cfg);

    printVehicleSummary(P);

    % MOI check
    if any(isnan(P.J(:))) || any(diag(P.J) <= 0)
        fprintf('\n[중단] MOI가 입력되지 않았습니다.\n');
        fprintf('CATIA에서 Ixx, Iyy, Izz를 kg*m^2 단위로 입력한 뒤 다시 실행하세요.\n');
        fprintf('현재 이 설계안은 prop/hover 정보까지만 계산했습니다.\n');

        results(k).P = P;
        continue;
    end

    % Hover trim
    [x0, u0, trim] = hoverTrim(P, cfg);

    fprintf('\n[Hover trim]\n');
    fprintf('Hover thrust per rotor : %.4f N = %.2f gf\n', ...
        trim.T_hover_each_N, trim.T_hover_each_gf);
    fprintf('omega_hover            : %.2f rad/s\n', trim.omega_hover);
    fprintf('rpm_hover              : %.0f RPM\n', trim.rpm_hover);
    fprintf('Total thrust           : %.4f N\n', 4 * trim.T_hover_each_N);
    fprintf('Weight                 : %.4f N\n', P.mass_kg * cfg.g);

    % 6-DOF numerical linearization
    [A, B] = linearizeNumerical(@(x,u) quad6dofModel(x, u, P, cfg), x0, u0);

    % Physical PID design
    pidPhys = designRatePIDPhysical(P, cfg);

    % Control authority around hover
    margin = computeControlMargin(P, cfg);

    % Convert physical moment PID to PX4 normalized rate PID
    px4 = convertToPX4Params(pidPhys, margin, cfg);

    % Print result
    printControlSummary(P, pidPhys, margin, px4, cfg);

    % Root Locus visualization
    plotRootLocus(P, pidPhys, cfg);

    if cfg.printStateSpace
        fprintf('\n[A matrix]\n');
        disp(A);

        fprintf('\n[B matrix]\n');
        disp(B);
    end

    % Save results
    results(k).P = P;
    results(k).x0 = x0;
    results(k).u0 = u0;
    results(k).trim = trim;
    results(k).A = A;
    results(k).B = B;
    results(k).pidPhys = pidPhys;
    results(k).margin = margin;
    results(k).px4 = px4;
end

fprintf('\n============================================================\n');
fprintf('Calculation finished.\n');
fprintf('results(1), results(2)에 각 설계안 계산 결과 저장 완료.\n');
fprintf('============================================================\n');

end % 메인 함수(control_warmstart)의 끝

%% ================================================================
% Local functions
% ================================================================

function design = fetchDesignFromWorkspace(cfg)
    fprintf('\n[Design input mode]\n');
    fprintf('Base Workspace에 있는 개념설계 결과값을 자동으로 불러옵니다...\n');

    % =========================
    % Design 1: Endurance
    % =========================
    design(1).name = 'Design 1 - Endurance';

    try
        mass   = evalin('base', 'final_aum_p');
        arm    = evalin('base', 'best_arm_len_p');
        hgt    = evalin('base', 'final_height_m');
        prop   = evalin('base', 'propSpecification');
        motor  = evalin('base', 'motorChosen');
        esc    = evalin('base', 'best_esc');
        cellNo = evalin('base', 'BattCellNo');
        cellV  = evalin('base', 'BattCellVoltage');

        design(1).mass_kg = toNumeric(mass) / 1000;
        design(1).arm_m = toNumeric(arm) / 1000;
        design(1).rotor_height_m = toNumeric(hgt);
        
        design(1).propTarget = char(prop{1});
        design(1).motorName = char(motor{2});
        design(1).motor_kv = toNumeric(motor{5});
        
        if ~isempty(esc)
            design(1).escName = char(esc{2});
        else
            design(1).escName = 'Not selected';
        end
        
        design(1).V_nom = toNumeric(cellNo) * toNumeric(cellV);
        design(1).tau_m = cfg.default_tau_m;
        design(1).source = 'Base Workspace (main.m 등 실행 결과)';

    catch ME
        error('\n[오류] Workspace에서 Design 1 변수를 찾을 수 없습니다.\n개념설계 코드를 먼저 실행해서 변수를 생성해주세요.\n상세: %s', ME.message);
    end

    % =========================
    % Design 2: Maneuverability
    % =========================
    design(2).name = 'Design 2 - Maneuverability';

    try
        prop_ff  = evalin('base', 'prop_ff');
        motor_ff = evalin('base', 'motorChosen_ff');
        esc_ff   = evalin('base', 'best_esc_ff');

        design(2).mass_kg = toNumeric(mass) / 1000; 
        design(2).arm_m = toNumeric(arm) / 1000;
        design(2).rotor_height_m = toNumeric(hgt);

        design(2).propTarget = char(prop_ff{1});
        design(2).motorName = char(motor_ff{2});
        design(2).motor_kv = toNumeric(motor_ff{5});

        if ~isempty(esc_ff)
            design(2).escName = char(esc_ff{2});
        else
            design(2).escName = 'Not selected';
        end
        
        design(2).V_nom = toNumeric(cellNo) * toNumeric(cellV);
        design(2).tau_m = cfg.default_tau_m;
        design(2).source = 'Base Workspace (forward_flight_analysis.m 실행 결과)';

    catch
        fprintf('\n[안내] 기동성 해석 변수(_ff)를 찾지 못해 Design 1과 동일하게 설정합니다.\n');
        design(2) = design(1);
        design(2).name = 'Design 2 - Maneuverability (Not Found)';
    end

    printExtractedDesignSummary(design);
end

function P = buildVehicleParams(d, propList, cfg)

    P.name = d.name;
    P.mass_kg = d.mass_kg;
    P.arm_m = d.arm_m;
    P.rotor_height_m = d.rotor_height_m;
    P.motor_kv = d.motor_kv;
    P.V_nom = d.V_nom;
    P.tau_m = d.tau_m;
    P.N = cfg.RotorNo;

    if isfield(d, 'motorName')
        P.motorName = d.motorName;
    else
        P.motorName = 'Unknown';
    end

    if isfield(d, 'escName')
        P.escName = d.escName;
    else
        P.escName = 'Unknown';
    end

    P.J = [
        d.Ixx, -d.Ixy, -d.Ixz;
       -d.Ixy,  d.Iyy, -d.Iyz;
       -d.Ixz, -d.Iyz,  d.Izz
    ];

    a = P.arm_m / sqrt(2);

    P.rotorPos_m = [
         a,  a, 0;    % M1: front-right, CCW
        -a, -a, 0;    % M2: rear-left,  CCW
         a, -a, 0;    % M3: front-left,  CW
        -a,  a, 0     % M4: rear-right, CW
    ];

    P.spinSign = [
         1;
         1;
        -1;
        -1
    ];

    propRow = findPropRow(propList, d.propTarget);

    P.propName = char(propRow{1});
    P.propFile = char(propRow{2});
    P.propDiameter_in = toNumeric(propRow{3});
    P.propPitch_in = toNumeric(propRow{4});
    P.propMass_g = toNumeric(propRow{5});
    P.propSpeedLimit_RPM = toNumeric(propRow{6});

    perf = load_propPerf(P.propFile, false);
    P.propPerf = perf;

    P.T_hover_each_N = P.mass_kg * cfg.g / P.N;
    P.T_hover_each_gf = P.T_hover_each_N / cfg.g * 1000;

    P.rpm_hover_from_data = interpRPMFromThrust(perf, P.T_hover_each_gf);

    [P.kT, P.kQ, fitInfo] = fitPropCoefficients(perf, P.rpm_hover_from_data, cfg);
    P.fitInfo = fitInfo;

    P.gamma = P.kQ / P.kT;

    P.omega_hover = sqrt(P.T_hover_each_N / P.kT);
    P.rpm_hover = P.omega_hover * cfg.RAD2RPM;

    rpm_motor_noload = P.motor_kv * P.V_nom * cfg.eta_noload;

    if isnan(P.propSpeedLimit_RPM) || P.propSpeedLimit_RPM <= 0
        P.rpm_max_est = rpm_motor_noload;
    else
        P.rpm_max_est = min(P.propSpeedLimit_RPM, rpm_motor_noload);
    end

    P.omega_max_est = P.rpm_max_est * cfg.RPM2RAD;

    P.Tmax_each_gf = interp1(perf(:,1), perf(:,2), P.rpm_max_est, 'linear', 'extrap');
    P.Tmax_each_N = P.Tmax_each_gf * cfg.g / 1000;
    P.Qmax_each_Nm = interp1(perf(:,1), perf(:,4), P.rpm_max_est, 'linear', 'extrap');

    P.hover_rpm_ratio = P.rpm_hover / P.rpm_max_est;
    P.thrust_to_weight_max = P.N * P.Tmax_each_N / (P.mass_kg * cfg.g);

    x = P.rotorPos_m(:,1);
    y = P.rotorPos_m(:,2);

    P.Aalloc = [
        ones(1,4);
        -y';
         x';
        (P.gamma * P.spinSign)'
    ];

    if abs(det(P.Aalloc)) < 1e-12
        error('%s: Allocation matrix is singular.', P.name);
    end

    P.Ainv = inv(P.Aalloc);
end

function rpmHover = interpRPMFromThrust(perf, thrust_gf)
    thrust = perf(:,2);
    rpm = perf(:,1);
    [thrustUnique, idx] = unique(thrust, 'stable');
    rpmUnique = rpm(idx);
    if thrust_gf < min(thrustUnique) || thrust_gf > max(thrustUnique)
        error('Hover thrust %.2f gf is outside APC prop data range [%.2f, %.2f] gf.', thrust_gf, min(thrustUnique), max(thrustUnique));
    end
    rpmHover = interp1(thrustUnique, rpmUnique, thrust_gf, 'linear');
end

function propRow = findPropRow(propList, target)
    targetKey = normalizePropName(target);
    names = string(propList(:,1));
    files = string(propList(:,2));
    idx = [];
    for i = 1:numel(names)
        nameKey = normalizePropName(names(i));
        fileKey = normalizePropName(files(i));
        if contains(nameKey, targetKey) || contains(fileKey, targetKey) || contains(targetKey, nameKey)
            idx = i;
            break;
        end
    end
    if isempty(idx)
        error('Propeller "%s" was not found in propList.', target);
    end
    propRow = propList(idx,:);
end

function key = normalizePropName(s)
    key = lower(regexprep(string(s), '[^a-zA-Z0-9]', ''));
end

function [kT, kQ, fitInfo] = fitPropCoefficients(perf, rpm_hover, cfg)
    rpm = perf(:,1);
    thrust_N = perf(:,2) * cfg.g / 1000;
    torque_Nm = perf(:,4);
    idx = rpm >= rpm_hover * (1 - cfg.fitBand) & rpm <= rpm_hover * (1 + cfg.fitBand);
    if sum(idx) < 4
        [~, order] = sort(abs(rpm - rpm_hover));
        idx = false(size(rpm));
        idx(order(1:min(6,numel(order)))) = true;
    end
    omega = rpm(idx) * cfg.RPM2RAD;
    X = omega.^2;
    kT = X \ thrust_N(idx);
    kQ = X \ torque_Nm(idx);
    if kT <= 0 || kQ <= 0
        error('Invalid fitted prop coefficients: kT=%.3e, kQ=%.3e', kT, kQ);
    end
    fitInfo.rpm_used = rpm(idx);
    fitInfo.thrust_N_used = thrust_N(idx);
    fitInfo.torque_Nm_used = torque_Nm(idx);
    fitInfo.kT = kT;
    fitInfo.kQ = kQ;
end

function [x0, u0, trim] = hoverTrim(P, cfg)
    T_hover_each_N = P.mass_kg * cfg.g / P.N;
    T_hover_each_gf = T_hover_each_N / cfg.g * 1000;
    omega_hover = sqrt(T_hover_each_N / P.kT);
    rpm_hover = omega_hover * cfg.RAD2RPM;
    x0 = zeros(16,1);
    x0(13:16) = omega_hover;
    u0 = omega_hover * ones(4,1);
    trim.T_hover_each_N = T_hover_each_N;
    trim.T_hover_each_gf = T_hover_each_gf;
    trim.omega_hover = omega_hover;
    trim.rpm_hover = rpm_hover;
end

function dx = quad6dofModel(x, ucmd, P, cfg)
    vB = x(1:3); wB = x(4:6);
    phi = x(7); theta = x(8); psi = x(9);
    omega = x(13:16);
    omega_cmd = max(ucmd(:), 0);
    omega = max(omega(:), 0);
    omega_dot = (omega_cmd - omega) / P.tau_m;
    T = P.kT * omega.^2;
    Q = P.kQ * omega.^2;
    F_thrust_B = [0; 0; -sum(T)];
    F_drag_B = [0; 0; 0];
    F_B = F_thrust_B + F_drag_B;
    M_B = zeros(3,1);
    for i = 1:4
        ri = P.rotorPos_m(i,:)';
        Fi = [0; 0; -T(i)];
        M_B = M_B + cross(ri, Fi);
    end
    M_B(3) = M_B(3) + sum(P.spinSign .* Q);
    R_nb = euler321_Rnb(phi, theta, psi);
    g_N = [0; 0; cfg.g];
    g_B = R_nb' * g_N;
    vB_dot = F_B / P.mass_kg + g_B - cross(wB, vB);
    J = P.J;
    wB_dot = J \ (M_B - cross(wB, J*wB));
    E = eulerRateMatrix(phi, theta);
    euler_dot = E * wB;
    pos_dot = R_nb * vB;
    dx = zeros(16,1);
    dx(1:3) = vB_dot; dx(4:6) = wB_dot; dx(7:9) = euler_dot;
    dx(10:12) = pos_dot; dx(13:16) = omega_dot;
end

function R = euler321_Rnb(phi, theta, psi)
    cphi = cos(phi); sphi = sin(phi);
    cth = cos(theta); sth = sin(theta);
    cpsi = cos(psi); spsi = sin(psi);
    R = [ cth*cpsi, sphi*sth*cpsi - cphi*spsi, cphi*sth*cpsi + sphi*spsi;
          cth*spsi, sphi*sth*spsi + cphi*cpsi, cphi*sth*spsi - sphi*cpsi;
          -sth,     sphi*cth,                  cphi*cth ];
end

function E = eulerRateMatrix(phi, theta)
    cphi = cos(phi); sphi = sin(phi); cth = cos(theta);
    if abs(cth) < 1e-6, error('Euler angle singularity'); end
    E = [ 1, sphi*tan(theta), cphi*tan(theta);
          0, cphi, -sphi;
          0, sphi/cth, cphi/cth ];
end

function [A, B] = linearizeNumerical(f, x0, u0)
    nx = numel(x0); nu = numel(u0);
    A = zeros(nx,nx); B = zeros(nx,nu);
    f0 = f(x0,u0);
    for i = 1:nx
        h = 1e-5 * max(1, abs(x0(i)));
        xp = x0; xm = x0;
        xp(i) = xp(i) + h; xm(i) = xm(i) - h;
        A(:,i) = (f(xp,u0) - f(xm,u0)) / (2*h);
    end
    for j = 1:nu
        h = 1e-5 * max(1, abs(u0(j)));
        up = u0; um = u0;
        up(j) = up(j) + h; um(j) = um(j) - h;
        B(:,j) = (f(x0,up) - f(x0,um)) / (2*h);
    end
end

function pid = designRatePIDPhysical(P, cfg)
    I = diag(P.J); tau = P.tau_m;
    wn = [cfg.wn_roll; cfg.wn_pitch; cfg.wn_yaw];
    zeta = cfg.zeta_rate;
    axisName = ["roll"; "pitch"; "yaw"];
    for i = 1:3
        p3_min = max(0, 1.2/tau - 2*zeta*wn(i));
        p3 = max(cfg.thirdPoleMultiplier * wn(i), p3_min);
        Kd = I(i)*tau*(2*zeta*wn(i) + p3) - I(i);
        Kp = I(i)*tau*(wn(i)^2 + 2*zeta*wn(i)*p3);
        Ki = I(i)*tau*(wn(i)^2 * p3);
        if Kd < 0
            Kd = 0; Kp = 2*zeta*wn(i)*I(i); Ki = wn(i)^2*I(i);
        end
        pid.axis(i).name = axisName(i);
        pid.axis(i).I = I(i); pid.axis(i).wn = wn(i); pid.axis(i).zeta = zeta;
        pid.axis(i).p3 = p3; pid.axis(i).Kp = Kp; pid.axis(i).Ki = Ki; pid.axis(i).Kd = Kd;
    end
end

function margin = computeControlMargin(P, cfg)
    T0 = P.mass_kg * cfg.g / 4; Tmin = 0; Tmax = P.Tmax_each_N;
    margin.roll_pos = axisMomentLimit(P, cfg, Tmin, Tmax, [0; 1; 0; 0]);
    margin.roll_neg = axisMomentLimit(P, cfg, Tmin, Tmax, [0; -1; 0; 0]);
    margin.pitch_pos = axisMomentLimit(P, cfg, Tmin, Tmax, [0; 0; 1; 0]);
    margin.pitch_neg = axisMomentLimit(P, cfg, Tmin, Tmax, [0; 0; -1; 0]);
    margin.yaw_pos = axisMomentLimit(P, cfg, Tmin, Tmax, [0; 0; 0; 1]);
    margin.yaw_neg = axisMomentLimit(P, cfg, Tmin, Tmax, [0; 0; 0; -1]);
    margin.Mscale_roll = min(margin.roll_pos, margin.roll_neg);
    margin.Mscale_pitch = min(margin.pitch_pos, margin.pitch_neg);
    margin.Mscale_yaw = min(margin.yaw_pos, margin.yaw_neg);
    I = diag(P.J);
    margin.roll_alpha_max = margin.Mscale_roll / I(1);
    margin.pitch_alpha_max = margin.Mscale_pitch / I(2);
    margin.yaw_alpha_max = margin.Mscale_yaw / I(3);
    dTlim = min(T0, Tmax - T0); a = P.arm_m / sqrt(2);
    margin.dTlim = dTlim;
    margin.Mscale_roll_simple = 4 * a * dTlim;
    margin.Mscale_pitch_simple = 4 * a * dTlim;
    margin.Mscale_yaw_simple = 4 * P.gamma * dTlim;
end

function Mlim = axisMomentLimit(P, cfg, Tmin, Tmax, virtualDirection)
    Uhover = P.mass_kg * cfg.g;
    lo = 0; hi = 10;
    for i = 1:30
        vcmd = [Uhover; 0; 0; 0] + hi * virtualDirection;
        Tcmd = P.Ainv * vcmd;
        if all(Tcmd >= Tmin) && all(Tcmd <= Tmax), hi = hi * 2; else, break; end
    end
    for n = 1:80
        mid = 0.5*(lo + hi);
        vcmd = [Uhover; 0; 0; 0] + mid * virtualDirection;
        Tcmd = P.Ainv * vcmd;
        if all(Tcmd >= Tmin) && all(Tcmd <= Tmax), lo = mid; else, hi = mid; end
    end
    Mlim = lo;
end

function px4 = convertToPX4Params(pidPhys, margin, cfg)
    Mscale = [margin.Mscale_roll; margin.Mscale_pitch; margin.Mscale_yaw];
    Kp_phys = [pidPhys.axis(1).Kp; pidPhys.axis(2).Kp; pidPhys.axis(3).Kp];
    Ki_phys = [pidPhys.axis(1).Ki; pidPhys.axis(2).Ki; pidPhys.axis(3).Ki];
    Kd_phys = [pidPhys.axis(1).Kd; pidPhys.axis(2).Kd; pidPhys.axis(3).Kd];
    sf = cfg.px4SafetyFactor;
    P_px4 = sf * Kp_phys ./ Mscale;
    I_px4 = sf * Ki_phys ./ Mscale;
    D_px4 = sf * Kd_phys ./ Mscale;
    
    px4.MC_ROLLRATE_K = 1.0; px4.MC_PITCHRATE_K = 1.0; px4.MC_YAWRATE_K = 1.0;
    px4.MC_ROLLRATE_P = P_px4(1); px4.MC_ROLLRATE_I = I_px4(1); px4.MC_ROLLRATE_D = D_px4(1);
    px4.MC_PITCHRATE_P = P_px4(2); px4.MC_PITCHRATE_I = I_px4(2); px4.MC_PITCHRATE_D = D_px4(2);
    px4.MC_YAWRATE_P = P_px4(3); px4.MC_YAWRATE_I = I_px4(3); px4.MC_YAWRATE_D = D_px4(3);
    px4.MC_ROLL_P = cfg.MC_ROLL_P; px4.MC_PITCH_P = cfg.MC_PITCH_P; px4.MC_YAW_P = cfg.MC_YAW_P;
    px4.safetyFactor = sf;
end

function printVehicleSummary(P)
    fprintf('\n[Propulsion / geometry]\n');
    fprintf('Propeller target        : %s\n', P.propName);
    fprintf('Propeller file          : %s\n', P.propFile);
    fprintf('Mass                    : %.4f kg\n', P.mass_kg);
    fprintf('Arm length              : %.3f m\n', P.arm_m);
    fprintf('Motor                   : %s\n', P.motorName);
    fprintf('Motor kV                : %.0f kV\n', P.motor_kv);
    fprintf('ESC                     : %s\n', P.escName);
    fprintf('Nominal voltage         : %.2f V\n', P.V_nom);
    fprintf('Motor time constant     : %.3f s\n', P.tau_m);
    fprintf('\n[Prop coefficient fitting near hover]\n');
    fprintf('Hover thrust            : %.2f gf/rotor\n', P.T_hover_each_gf);
    fprintf('Hover RPM from data     : %.0f RPM\n', P.rpm_hover_from_data);
    fprintf('Hover RPM from kT       : %.0f RPM\n', P.rpm_hover);
    fprintf('kT                      : %.6e N/(rad/s)^2\n', P.kT);
    fprintf('kQ                      : %.6e N*m/(rad/s)^2\n', P.kQ);
    fprintf('gamma = kQ/kT           : %.6e m\n', P.gamma);
end

function printControlSummary(P, pidPhys, margin, px4, cfg)
    fprintf('\n[MOI]\n');
    fprintf('Ixx = %.6e kg*m^2\n', P.J(1,1));
    fprintf('Iyy = %.6e kg*m^2\n', P.J(2,2));
    fprintf('Izz = %.6e kg*m^2\n', P.J(3,3));
    fprintf('\n[Physical rate PID warm-start]\n');
    for i = 1:3
        fprintf('%s-rate: Kp = %.6e, Ki = %.6e, Kd = %.6e, wn = %.2f rad/s\n', pidPhys.axis(i).name, pidPhys.axis(i).Kp, pidPhys.axis(i).Ki, pidPhys.axis(i).Kd, pidPhys.axis(i).wn);
    end
    fprintf('\n[PX4 warm-start parameters]\n');
    fprintf('Safety factor applied: %.2f\n', cfg.px4SafetyFactor);
    fprintf('\n# Rate controller\n');
    fprintf('MC_ROLLRATE_P    = %.6f\n', px4.MC_ROLLRATE_P);
    fprintf('MC_ROLLRATE_I    = %.6f\n', px4.MC_ROLLRATE_I);
    fprintf('MC_ROLLRATE_D    = %.6f\n', px4.MC_ROLLRATE_D);
    fprintf('MC_PITCHRATE_P   = %.6f\n', px4.MC_PITCHRATE_P);
    fprintf('MC_PITCHRATE_I   = %.6f\n', px4.MC_PITCHRATE_I);
    fprintf('MC_PITCHRATE_D   = %.6f\n', px4.MC_PITCHRATE_D);
    fprintf('MC_YAWRATE_P     = %.6f\n', px4.MC_YAWRATE_P);
    fprintf('MC_YAWRATE_I     = %.6f\n', px4.MC_YAWRATE_I);
    fprintf('MC_YAWRATE_D     = %.6f\n', px4.MC_YAWRATE_D);
    fprintf('\n# Attitude controller\n');
    fprintf('MC_ROLL_P        = %.6f\n', px4.MC_ROLL_P);
    fprintf('MC_PITCH_P       = %.6f\n', px4.MC_PITCH_P);
    fprintf('MC_YAW_P         = %.6f\n', px4.MC_YAW_P);
    fprintf('\n[Important]\n실제 적용 전 motor order, spin direction, yaw sign, actuator output을 반드시 확인하세요.\n');
end

function printExtractedDesignSummary(design)
    fprintf('\n[Design Summary]\n');
    fprintf('%s\n', repmat('-', 1, 92));
    fprintf('%-28s %-16s %-12s %-12s %-12s %-10s\n', 'Design', 'Prop', 'Mass[kg]', 'Arm[m]', 'Height[m]', 'Motor kV');
    fprintf('%s\n', repmat('-', 1, 92));
    for i = 1:numel(design)
        fprintf('%-28s %-16s %-12.4f %-12.4f %-12.4f %-10.0f\n', design(i).name, design(i).propTarget, design(i).mass_kg, design(i).arm_m, design(i).rotor_height_m, design(i).motor_kv);
    end
    fprintf('%s\n', repmat('-', 1, 92));
    fprintf('\nMOI는 아직 NaN입니다. CATIA 계산값을 입력한 뒤 6-DOF/PX4 gain 계산을 진행하세요.\n');
end

function val = toNumeric(x)
    if isnumeric(x), val = x; return; end
    if isstring(x) || ischar(x), val = str2double(x); return; end
    if iscell(x), val = toNumeric(x{1}); return; end
    error('Cannot convert value to numeric.');
end

%% ================================================================
% plotRootLocus  —  Root Locus + 폐루프 극점 시각화
%
% 각 축(Roll / Pitch / Yaw)에 대해:
%   1) 플랜트 전달함수  G(s) = 1 / (I·s · (τ·s + 1))
%      - 입력: 토크(τ_cmd), 출력: 각속도(ω)  [Rate loop 관점]
%      - 모터 1차 지연 (τ_m) 포함
%
%   2) PID 컨트롤러  C(s) = Kp + Ki/s + Kd·s
%      - 실제 구현: C(s) = (Kd·s² + Kp·s + Ki) / s
%
%   3) 개루프  L(s) = C(s)·G(s)
%      - Kp 를 0 → 3×Kp_설계 로 스윕 → Root Locus 궤적
%      - 나머지 Ki, Kd 는 설계값 고정
%
%   4) 그래프 위에 표시
%      - Root Locus 궤적 (회색)
%      - 설계 Kp 에서의 폐루프 극점 ★
%      - 목표 극점 위치  ◎  (−ζ·wn ± j·wn·√(1−ζ²))
%      - ζ=0.8 감쇠비 선 (녹색 점선)
%      - 안정 경계 (허수축, 빨간 점선)
%% ================================================================
function plotRootLocus(P, pidPhys, cfg)

    axisLabel = ["Roll", "Pitch", "Yaw"];
    colors     = {'#1A6FBF', '#B84A00', '#1A8C4E'};  % blue / coral / green
    tau = P.tau_m;
    I   = diag(P.J);

    fig = figure('Name', sprintf('[%s] Root Locus — Rate PID', P.name), ...
                 'NumberTitle', 'off', ...
                 'Position', [100, 80, 1320, 480]);

    for ax_idx = 1:3

        pid_ax = pidPhys.axis(ax_idx);
        Ii     = I(ax_idx);
        wn     = pid_ax.wn;
        zeta   = pid_ax.zeta;
        Kp0    = pid_ax.Kp;
        Ki0    = pid_ax.Ki;
        Kd0    = pid_ax.Kd;

        % ── 플랜트: G(s) = 1 / (Ii·s·(tau·s+1)) ─────────────────────
        %   토크입력 → 각속도 출력 (Rate loop)
        %   분모: Ii·tau·s² + Ii·s
        G = tf(1, [Ii*tau, Ii, 0]);

        % ── PID: C(s) = (Kd·s² + Kp·s + Ki) / s ─────────────────────
        %   Ki0, Kd0 를 설계값으로 고정, Kp 만 스윕할 개루프 만들기
        %   L(s) = C(s)·G(s) = (Kd·s² + Kp·s + Ki)·G(s) / s
        %
        %   Root Locus 는  L(s) = Kp · G_rlocus(s)  형태가 필요하므로
        %   Kp 이외 항을 플랜트에 흡수한 등가 개루프를 구성
        %
        %   L_eq(s) = G(s)/s · (Kd·s² + Ki)   ← Kp 제외 고정 부분
        %   + Kp · G(s)/s                        ← Kp 스윕 부분
        %
        %   즉  L(s) = Kp·[G(s)/s] + L_fixed(s)
        %   rlocus 는  단일 gain k 에 대한 궤적이므로
        %   등가 개루프: G_eq(s) = G(s)/s  (Kp 를 루트로커스 gain 으로 사용)
        %   단, 이 방법은 Ki·Kd 효과를 근사하므로 실제 폐루프 극점은
        %   별도로 정확히 계산해 ★ 로 표시함

        % Kp 스윕용 등가 개루프 (Ki=Kd=0 로 두고 Kp 만 변화시키는 단순화)
        G_int = tf(1, [Ii*tau, Ii, 0, 0]);   % G(s)/s = 1/(Ii·tau·s³+Ii·s²)

        % ── Root Locus 궤적 계산 ──────────────────────────────────────
        Kp_sweep = linspace(0, 3*Kp0, 500);
        rl_poles = zeros(numel(Kp_sweep), order(G_int));
        [num_G, den_G] = tfdata(G_int, 'v');
        for ki = 1:numel(Kp_sweep)
            cl_den = den_G + Kp_sweep(ki) * num_G;
            rl_poles(ki, :) = roots(cl_den);
        end

        % ── 설계 게인에서 실제 폐루프 극점 (정확한 계산) ─────────────
        %   C(s)·G(s) = (Kd·s²+Kp·s+Ki)·G(s)/s
        %   분자: Kd·s² + Kp·s + Ki
        %   분모_G: Ii·tau·s² + Ii·s  → G(s)/s 분모: Ii·tau·s³ + Ii·s²
        C_num = [Kd0, Kp0, Ki0];
        C_den = [1, 0];
        CG_num = conv(C_num, [1]);        % C분자 × G분자(=1)
        CG_den = conv(C_den, [Ii*tau, Ii, 0]);  % C분모 × G분모
        % 폐루프 특성방정식: CG_den + CG_num = 0
        cl_char = CG_den;
        cl_char(end-length(CG_num)+1:end) = cl_char(end-length(CG_num)+1:end) + CG_num;
        cl_poles_exact = roots(cl_char);

        % ── 목표 극점 위치 ─────────────────────────────────────────────
        sigma_d = zeta * wn;
        wd      = wn * sqrt(1 - zeta^2);
        p_target = [-sigma_d + 1j*wd; -sigma_d - 1j*wd];

        % ── 축 범위 결정 (극점들 기준으로 자동) ──────────────────────
        all_re = [real(rl_poles(:)); real(cl_poles_exact); real(p_target)];
        all_im = [imag(rl_poles(:)); imag(cl_poles_exact); imag(p_target)];
        pad = 0.15;
        re_min = min(all_re) * (1 + pad);  re_max = max(0.5, max(all_re)) * (1 + pad);
        im_max = max(abs(all_im)) * (1 + pad);
        if im_max < 1, im_max = wn * 1.5; end
        re_range = [max(-6*wn, re_min), min(2, re_max)];
        im_range = [-im_max, im_max];

        subplot(1, 3, ax_idx);
        hold on; grid on; box on;
        set(gca, 'FontSize', 9);

        % 안정 경계 (허수축)
        plot([0 0], im_range, 'r--', 'LineWidth', 0.8, 'DisplayName', '안정 경계');

        % ζ=0.8 감쇠비 선
        zeta_angle = acos(zeta);
        r_line = im_max * 1.2;
        plot([0, -r_line*cos(zeta_angle)], [0,  r_line*sin(zeta_angle)], ...
            'Color', '#1A8C4E', 'LineStyle', '--', 'LineWidth', 0.9, 'DisplayName', sprintf('\\zeta = %.2f', zeta));
        plot([0, -r_line*cos(zeta_angle)], [0, -r_line*sin(zeta_angle)], ...
            'Color', '#1A8C4E', 'LineStyle', '--', 'LineWidth', 0.9, 'HandleVisibility', 'off');

        % Root Locus 궤적 (Kp 스윕)
        n_ord = size(rl_poles, 2);
        for br = 1:n_ord
            re_v = real(rl_poles(:, br));
            im_v = imag(rl_poles(:, br));
            % 축 범위 밖 클리핑
            mask = re_v >= re_range(1) & re_v <= re_range(2) & ...
                   im_v >= im_range(1) & im_v <= im_range(2);
            if br == 1
                plot(re_v(mask), im_v(mask), '.', 'Color', [0.7 0.7 0.7], ...
                    'MarkerSize', 2, 'DisplayName', 'Root Locus (K_p 스윕)');
            else
                plot(re_v(mask), im_v(mask), '.', 'Color', [0.7 0.7 0.7], ...
                    'MarkerSize', 2, 'HandleVisibility', 'off');
            end
        end

        % 개루프 극점 (×)
        ol_poles = roots([Ii*tau, Ii, 0, 0]);
        plot(real(ol_poles), imag(ol_poles), 'x', ...
            'Color', '#444444', 'MarkerSize', 9, 'LineWidth', 1.4, ...
            'DisplayName', '개루프 극점');

        % 목표 극점 (◎)
        plot(real(p_target), imag(p_target), 'o', ...
            'Color', '#1A8C4E', 'MarkerSize', 10, 'LineWidth', 1.6, ...
            'DisplayName', sprintf('목표 극점 \\omega_n=%.1f', wn));

        % 실제 폐루프 극점 (★)
        plot(real(cl_poles_exact), imag(cl_poles_exact), 'p', ...
            'Color', colors{ax_idx}, 'MarkerSize', 13, 'LineWidth', 1.5, ...
            'MarkerFaceColor', colors{ax_idx}, ...
            'DisplayName', sprintf('폐루프 극점 (설계 K_p=%.3g)', Kp0));

        % 극점 좌표 텍스트 레이블
        for pi = 1:numel(cl_poles_exact)
            p = cl_poles_exact(pi);
            re_p = real(p); im_p = imag(p);
            if re_p >= re_range(1) && re_p <= re_range(2) && ...
               im_p >= im_range(1) && im_p <= im_range(2)
                if im_p >= 0
                    txt = sprintf('  %.2f+%.2fj', re_p, im_p);
                else
                    txt = sprintf('  %.2f%.2fj', re_p, im_p);
                end
                text(re_p, im_p, txt, 'FontSize', 7.5, 'Color', colors{ax_idx}, ...
                    'VerticalAlignment', 'bottom');
            end
        end

        % 안정 여유 텍스트 (좌상단)
        % Gain Margin: 개루프가 허수축을 건너는 Kp 값
        gm_kp = NaN;
        for ki = 2:numel(Kp_sweep)
            re_now = real(rl_poles(ki,:));
            re_prev = real(rl_poles(ki-1,:));
            if any(re_prev < 0 & re_now >= 0)
                gm_kp = Kp_sweep(ki);
                break;
            end
        end
        if ~isnan(gm_kp) && gm_kp > Kp0
            gm_ratio = gm_kp / Kp0;
            gm_str = sprintf('Gain Margin: ×%.1f (K_p 한계 ≈ %.3g)', gm_ratio, gm_kp);
        else
            gm_str = 'Gain Margin: 스윕 범위 내 불안정 전환 없음';
        end
        annotation_str = {
            sprintf('K_p = %.4g  K_i = %.4g  K_d = %.4g', Kp0, Ki0, Kd0), ...
            gm_str
        };
        text(re_range(1)*0.92, im_range(2)*0.92, annotation_str, ...
            'FontSize', 7.5, 'VerticalAlignment', 'top', ...
            'BackgroundColor', [1 1 1 0.7], 'EdgeColor', [0.8 0.8 0.8]);

        % 축 설정
        xlim(re_range); ylim(im_range);
        xlabel('Real axis  (σ)', 'FontSize', 9);
        ylabel('Imaginary axis  (jω)', 'FontSize', 9);
        title(sprintf('%s — Rate PID Root Locus', axisLabel(ax_idx)), ...
              'FontSize', 10, 'FontWeight', 'bold');
        legend('Location', 'southeast', 'FontSize', 7.5);

        hold off;
    end

    sgtitle(sprintf('Root Locus 분석 — %s  (wn = [%.1f, %.1f, %.1f] rad/s, ζ = %.2f)', ...
        P.name, cfg.wn_roll, cfg.wn_pitch, cfg.wn_yaw, cfg.zeta_rate), ...
        'FontSize', 11, 'FontWeight', 'bold');

    fprintf('\n[Root Locus] 그림이 생성되었습니다: "%s"\n', ...
        get(fig, 'Name'));
    fprintf('  ★ 별 = 설계 게인에서의 실제 폐루프 극점\n');
    fprintf('  ◎ 원 = 목표 극점 위치 (wn/zeta 기반)\n');
    fprintf('  -- 녹선 = 감쇠비 zeta=%.2f 경계\n', cfg.zeta_rate);
    fprintf('  회색점선 = Kp 를 0→3×설계값으로 스윕한 Root Locus 궤적\n\n');

    % 3개 축 폐루프 극점 출력
    fprintf('%-8s  %-30s  %-12s  %-12s\n', '축', '폐루프 극점', '|σ| (감쇠)', '|ωd| (진동)');
    fprintf('%s\n', repmat('-', 1, 68));
    for ax_idx = 1:3
        pid_ax = pidPhys.axis(ax_idx);
        Ii = I(ax_idx);
        Kp0 = pid_ax.Kp; Ki0 = pid_ax.Ki; Kd0 = pid_ax.Kd;
        C_num = [Kd0, Kp0, Ki0];
        C_den = [1, 0];
        CG_den = conv(C_den, [Ii*tau, Ii, 0]);
        CG_num = C_num;
        cl_char = CG_den;
        cl_char(end-length(CG_num)+1:end) = cl_char(end-length(CG_num)+1:end) + CG_num;
        poles = roots(cl_char);
        for pi = 1:numel(poles)
            p = poles(pi);
            if pi == 1
                fprintf('%-8s  %+8.4f %+8.4fj              %8.4f      %8.4f\n', ...
                    axisLabel(ax_idx), real(p), imag(p), abs(real(p)), abs(imag(p)));
            else
                fprintf('%-8s  %+8.4f %+8.4fj              %8.4f      %8.4f\n', ...
                    ' ', real(p), imag(p), abs(real(p)), abs(imag(p)));
            end
        end
        fprintf('\n');
    end
end