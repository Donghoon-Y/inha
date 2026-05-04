%% =========================================================================
% Arm Wake Interference Analysis
%
% 실행 순서:
%   [1안 기준]  main → arm_wake_analysis
%   [2안 기준]  main → forward_flight_analysis → arm_wake_analysis
%
% 목적:
%   선정된 프롭 기준으로 arm 형상/두께/위치 범위 전체를 계산하여
%   공력 손실 최소 후보 및 허용 손실률 이내 최대 두께를 도출.
%
% 이론:
%   Actuator disk theory, hover condition
%     v_i     = sqrt(T / (2*rho*A))
%     v_wake  = k * v_i          (k: 위치에 따른 wake 속도 계수)
%     D_arm   = 0.5*rho*v_wake^2*Cd*A_proj
%     DeltaP  = arm 손실 보상을 위한 추가 기계출력 [W]
%
% 참조하는 Workspace 변수 (main.m 출력 필수):
%   propSpecification   — 1안 프롭 사양
%   operatingPoints     — 후보 프롭별 운용점
%   temp_propChosen_pos — 1안 선정 프롭 인덱스
%   RotorNo             — 로터 수 (전기출력 합산용)
%   motorChosen         — 선정 모터 (전기 파워 참조, 없으면 NaN 처리)
%
% 참조하는 Workspace 변수 (forward_flight_analysis.m 출력, 있으면 2안 우선):
%   prop_ff             — 2안 프롭 사양 (1×6 cell)
%   idx_ff_best         — 2안 프롭 인덱스 (operatingPoints 참조용)
%   motorChosen_ff      — 2안 모터 (전기 파워 참조)
% =========================================================================

%% 사전 확인: 필수 변수 존재 여부
required_vars = {'propSpecification', 'operatingPoints', ...
                 'temp_propChosen_pos', 'RotorNo'};
missing = {};
for v = required_vars
    if ~exist(v{1}, 'var'), missing{end+1} = v{1}; end
end
if ~isempty(missing)
    error('arm_wake_analysis: 다음 변수가 없습니다. main.m을 먼저 실행하세요.\n  누락: %s', ...
        strjoin(missing, ', '));
end

%% 물리 상수
rho   = 1.225;     % 공기 밀도 [kg/m^3]
g_acc = 9.80665;   % 중력가속도 [m/s^2]
IN2M  = 0.0254;    % inch -> m

%% 설계 파라미터
targetLossPct        = 3.0;        % 허용 추력 손실률 [%]
armThickness_mm_list = 5:1:40;     % arm 두께 후보 [mm]
yOverR_list          = 0:0.05:0.20;% arm 중심선 offset (y/R=0: 로터 중심 통과)
zOverR_list          = 0.05:0.05:1.00; % arm 수직 위치 (z/R=0.05: 디스크 바로 아래)
wakeCrossMode        = "radius";   % "radius" 또는 "diameter"
St                   = 0.20;       % Strouhal number (원형 실린더 근사)
bladeNo              = 2;          % 프롭 블레이드 수

% 형상별 대표 항력계수 (1차 근사, 실제 값은 Re/표면/자세에 따라 다름)
shapeName = ["circular"; "square"; "elliptic"; "streamlined"];
Cd_list   = [1.10;       1.80;     0.50;       0.15];

%% 분석 대상 프롭 및 운용점 선택
% prop_ff(2안)가 있으면 2안 프롭 기준으로 분석,
% 없으면 1안 프롭(temp_propChosen_pos) 기준으로 분석.
if exist('prop_ff','var') && ~isempty(prop_ff) && ...
   exist('idx_ff_best','var') && ~isempty(idx_ff_best)
    analysis_mode  = '2안 (기동성 최적 프롭)';
    prop_name      = string(prop_ff{1});
    prop_diam_in   = prop_ff{3};
    prop_pitch_in  = prop_ff{4};
    prop_idx       = idx_ff_best;

    % 2안 모터 전기 파워
    if exist('motorChosen_ff','var') && ~isempty(motorChosen_ff) && numel(motorChosen_ff) >= 11
        P_hover_el = motorChosen_ff{11};
    elseif exist('motorChosen','var') && numel(motorChosen) >= 11
        P_hover_el = motorChosen{11};
    else
        P_hover_el = NaN;
    end
else
    analysis_mode  = '1안 (호버 최적 프롭)';
    prop_name      = string(propSpecification{1});
    prop_diam_in   = propSpecification{2};
    prop_pitch_in  = propSpecification{3};
    prop_idx       = temp_propChosen_pos;

    % 1안 모터 전기 파워
    if exist('motorChosen','var') && numel(motorChosen) >= 11
        P_hover_el = motorChosen{11};
    else
        P_hover_el = NaN;
    end
end

%% 프롭 및 호버 운용점
D      = prop_diam_in * IN2M;
R      = D / 2;
A_disk = pi * R^2;

op_hover     = operatingPoints{prop_idx, 1};
RPM_hover    = op_hover(1);   % [rpm]
T_hover_gf   = op_hover(2);   % [gf/rotor]
Q_hover      = op_hover(3);   % [Nm/rotor]
P_hover_mech = op_hover(4);   % [W/rotor], APC 실측 기계 파워

T_hover_N = T_hover_gf * g_acc / 1000;   % [N/rotor]
v_i       = sqrt(T_hover_N / (2 * rho * A_disk));
f_rotor   = RPM_hover / 60;
f_BPF     = bladeNo * f_rotor;

%% 헤더 출력
fprintf('\n============================================================\n');
fprintf('[Arm Wake Interference Analysis]\n');
fprintf('Analysis mode            : %s\n', analysis_mode);
fprintf('============================================================\n');
fprintf('Selected propeller       : %s\n', prop_name);
fprintf('Propeller size           : %.1f x %.1f inch\n', prop_diam_in, prop_pitch_in);
fprintf('Propeller diameter D     : %.4f m\n', D);
fprintf('Propeller radius R       : %.4f m\n', R);
fprintf('Disk area A              : %.5f m^2\n', A_disk);
fprintf('Hover RPM                : %.0f rpm\n', RPM_hover);
fprintf('Hover thrust/rotor       : %.1f gf = %.3f N\n', T_hover_gf, T_hover_N);
fprintf('Hover torque/rotor       : %.4f Nm\n', Q_hover);
fprintf('Hover mech. power/rotor  : %.2f W  (APC 실측)\n', P_hover_mech);
if ~isnan(P_hover_el)
    fprintf('Hover elec. power/rotor  : %.2f W\n', P_hover_el);
end
fprintf('Induced velocity v_i     : %.3f m/s\n', v_i);
fprintf('Rotor frequency          : %.2f Hz\n', f_rotor);
fprintf('Blade passing frequency  : %.2f Hz\n', f_BPF);
fprintf('Target thrust loss       : %.2f %%\n', targetLossPct);
fprintf('Wake crossing mode       : %s\n', wakeCrossMode);
fprintf('============================================================\n');

%% 전체 파라미터 범위 계산
rows = {};
row  = 1;

for ss = 1:length(shapeName)
    shape = shapeName(ss);
    Cd    = Cd_list(ss);

    for dd = 1:length(armThickness_mm_list)
        d_mm = armThickness_mm_list(dd);

        for yy = 1:length(yOverR_list)
            yOverR = yOverR_list(yy);

            for zz = 1:length(zOverR_list)
                zOverR = zOverR_list(zz);

                out = calcArmWakeResult( ...
                    D, R, A_disk, T_hover_N, P_hover_mech, P_hover_el, ...
                    RPM_hover, bladeNo, rho, St, ...
                    shape, Cd, d_mm, yOverR, zOverR, ...
                    wakeCrossMode, targetLossPct);

                rows(row,:) = { ...
                    out.Shape, out.Cd, out.ArmThickness_mm, ...
                    out.y_over_R, out.y_mm, ...
                    out.z_over_R, out.z_mm, out.k, ...
                    out.L_wake_mm, out.A_proj_cm2, ...
                    out.Blockage_pct, out.v_i, out.v_wake, ...
                    out.D_arm_N, out.T_net_N, ...
                    out.ThrustLoss_pct, ...
                    out.P_new_mech_W, out.DeltaP_mech_W, out.DeltaP_mech_pct, ...
                    out.P_new_el_W, out.DeltaP_el_W, out.DeltaP_el_pct, ...
                    out.f_shed_Hz, out.f_rotor_Hz, out.f_BPF_Hz, ...
                    out.Diff_to_rotor_Hz, out.Diff_to_BPF_Hz, ...
                    out.Acceptable};
                row = row + 1;
            end
        end
    end
end

resultTable = cell2table(rows, 'VariableNames', { ...
    'Shape','Cd','ArmThickness_mm', ...
    'y_over_R','y_mm', ...
    'z_over_R','z_mm','k', ...
    'L_wake_mm','A_proj_cm2', ...
    'Blockage_pct','v_i','v_wake', ...
    'D_arm_N','T_net_N', ...
    'ThrustLoss_pct', ...
    'P_new_mech_W','DeltaP_mech_W','DeltaP_mech_pct', ...
    'P_new_el_W','DeltaP_el_W','DeltaP_el_pct', ...
    'f_shed_Hz','f_rotor_Hz','f_BPF_Hz', ...
    'Diff_to_rotor_Hz','Diff_to_BPF_Hz', ...
    'Acceptable'});

resultTable = sortrows(resultTable, {'ThrustLoss_pct','DeltaP_mech_W'});

%% 공력 손실 최소 후보
bestAeroCandidate = resultTable(1,:);
fprintf('\n공력 손실 최소 후보\n');
disp(bestAeroCandidate(:, {'Shape','Cd','ArmThickness_mm','y_over_R','y_mm', ...
    'z_over_R','z_mm','ThrustLoss_pct','DeltaP_mech_W','DeltaP_mech_pct', ...
    'Blockage_pct','v_wake','Acceptable'}));

%% 허용 손실률 이내 후보
acceptableTable = resultTable(resultTable.Acceptable == true, :);

if isempty(acceptableTable)
    warning('허용 손실률 %.2f%% 이내의 후보가 없습니다.', targetLossPct);
else
    % 선정 우선순위: 허용 이내 > arm 두께 최대 > 추력 손실 최소 > 추가출력 최소
    acceptableTable.SortThicknessNeg = -acceptableTable.ArmThickness_mm;
    acceptableSorted = sortrows(acceptableTable, ...
        {'SortThicknessNeg','ThrustLoss_pct','DeltaP_mech_W'});
    bestPracticalCandidate = acceptableSorted(1,:);
    bestPracticalCandidate.SortThicknessNeg = [];

    fprintf('\n허용 손실률 이내 최대 두께 후보\n');
    disp(bestPracticalCandidate(:, {'Shape','Cd','ArmThickness_mm','y_over_R','y_mm', ...
        'z_over_R','z_mm','ThrustLoss_pct','DeltaP_mech_W','DeltaP_mech_pct', ...
        'Blockage_pct','v_wake','Acceptable'}));
end

%% 형상별 최적 후보 (허용 이내 최대 두께 우선)
shapeBestRows = {};
for ss = 1:length(shapeName)
    shape = shapeName(ss);
    sub   = acceptableTable(acceptableTable.Shape == shape, :);

    if isempty(sub)
        shapeBestRows(end+1,:) = {shape, Cd_list(ss), NaN, NaN, NaN, NaN, NaN, ...
                                  NaN, NaN, NaN, false};
        continue;
    end

    sub.SortThicknessNeg = -sub.ArmThickness_mm;
    sub = sortrows(sub, {'SortThicknessNeg','ThrustLoss_pct','DeltaP_mech_W'});
    tmp = sub(1,:);

    shapeBestRows(end+1,:) = { ...
        tmp.Shape(1), tmp.Cd(1), tmp.ArmThickness_mm(1), ...
        tmp.y_over_R(1), tmp.y_mm(1), ...
        tmp.z_over_R(1), tmp.z_mm(1), ...
        tmp.ThrustLoss_pct(1), tmp.DeltaP_mech_W(1), ...
        tmp.Blockage_pct(1), true};
end

shapeBestTable = cell2table(shapeBestRows, 'VariableNames', ...
    {'Shape','Cd','BestArmThickness_mm','Best_y_over_R','Best_y_mm', ...
     'Best_z_over_R','Best_z_mm','ThrustLoss_pct','DeltaP_mech_W', ...
     'Blockage_pct','HasAcceptableCandidate'});

fprintf('\n형상별 최적 후보 — 허용 손실률 이내 최대 두께 우선\n');
disp(shapeBestTable);

%% 형상별 최대 허용 arm 두께
maxThicknessRows = {};
for ss = 1:length(shapeName)
    shape = shapeName(ss);
    sub   = acceptableTable(acceptableTable.Shape == shape, :);

    if isempty(sub)
        maxThicknessRows(end+1,:) = {shape, Cd_list(ss), NaN, NaN, NaN, NaN, ...
                                     NaN, NaN, false};
        continue;
    end

    maxD   = max(sub.ArmThickness_mm);
    subMax = sortrows(sub(sub.ArmThickness_mm == maxD, :), ...
                      {'ThrustLoss_pct','DeltaP_mech_W'});
    tmp    = subMax(1,:);

    maxThicknessRows(end+1,:) = { ...
        tmp.Shape(1), tmp.Cd(1), tmp.ArmThickness_mm(1), ...
        tmp.y_over_R(1), tmp.y_mm(1), ...
        tmp.z_over_R(1), tmp.z_mm(1), ...
        tmp.ThrustLoss_pct(1), true};
end

maxThicknessTable = cell2table(maxThicknessRows, 'VariableNames', ...
    {'Shape','Cd','MaxAllowableThickness_mm','y_over_R','y_mm', ...
     'z_over_R','z_mm','ThrustLoss_pct','HasAcceptableCandidate'});

%% 두께별 최소 필요 offset
offsetRows = {};
for ss = 1:length(shapeName)
    shape = shapeName(ss);

    for dd = 1:length(armThickness_mm_list)
        d_mm = armThickness_mm_list(dd);
        sub  = acceptableTable(acceptableTable.Shape == shape & ...
                               acceptableTable.ArmThickness_mm == d_mm, :);

        if isempty(sub)
            offsetRows(end+1,:) = {shape, d_mm, NaN, NaN, NaN, NaN, NaN, false};
        else
            sub = sortrows(sub, {'y_over_R','ThrustLoss_pct','DeltaP_mech_W'});
            tmp = sub(1,:);
            offsetRows(end+1,:) = { ...
                tmp.Shape(1), tmp.ArmThickness_mm(1), ...
                tmp.y_over_R(1), tmp.y_mm(1), ...
                tmp.z_over_R(1), tmp.z_mm(1), ...
                tmp.ThrustLoss_pct(1), true};
        end
    end
end

minOffsetTable = cell2table(offsetRows, 'VariableNames', ...
    {'Shape','ArmThickness_mm','Minimum_y_over_R','Minimum_y_mm', ...
     'z_over_R','z_mm','ThrustLoss_pct','HasAcceptableCandidate'});

%% z/R 영향 분석
zRows = {};
for ss = 1:length(shapeName)
    shape = shapeName(ss);

    for zz = 1:length(zOverR_list)
        zOverR = zOverR_list(zz);
        sub    = resultTable(resultTable.Shape == shape & ...
                             resultTable.z_over_R == zOverR, :);
        sub    = sortrows(sub, {'ThrustLoss_pct','DeltaP_mech_W'});
        tmp    = sub(1,:);

        zRows(end+1,:) = { ...
            tmp.Shape(1), tmp.z_over_R(1), tmp.z_mm(1), ...
            tmp.ArmThickness_mm(1), tmp.y_over_R(1), tmp.y_mm(1), ...
            tmp.ThrustLoss_pct(1), tmp.DeltaP_mech_W(1), tmp.Acceptable(1)};
    end
end

zEffectTable = cell2table(zRows, 'VariableNames', ...
    {'Shape','z_over_R','z_mm','BestArmThickness_mm','Best_y_over_R', ...
     'Best_y_mm','MinThrustLoss_pct','DeltaP_mech_W','Acceptable'});

%% 최종 요약 출력
fprintf('\n============================================================\n');
fprintf('[최종 요약]  분석 기준: %s\n', analysis_mode);
fprintf('============================================================\n');

fprintf('\n1) 순수 공력 손실 최소 후보\n');
fprintf('   Shape=%s, arm=%.1fmm, y/R=%.2f, z/R=%.2f\n', ...
    bestAeroCandidate.Shape, bestAeroCandidate.ArmThickness_mm, ...
    bestAeroCandidate.y_over_R, bestAeroCandidate.z_over_R);
fprintf('   Thrust loss=%.4f%%, DeltaP=%.4f W/rotor\n', ...
    bestAeroCandidate.ThrustLoss_pct, bestAeroCandidate.DeltaP_mech_W);

if ~isempty(acceptableTable)
    fprintf('\n2) 허용 손실률 %.1f%% 이내 최대 두께 후보\n', targetLossPct);
    fprintf('   Shape=%s, arm=%.1fmm, y/R=%.2f, z/R=%.2f\n', ...
        bestPracticalCandidate.Shape, bestPracticalCandidate.ArmThickness_mm, ...
        bestPracticalCandidate.y_over_R, bestPracticalCandidate.z_over_R);
    fprintf('   Thrust loss=%.4f%%, DeltaP=%.4f W/rotor\n', ...
        bestPracticalCandidate.ThrustLoss_pct, bestPracticalCandidate.DeltaP_mech_W);
    if ~isnan(P_hover_el)
        fprintf('   추가 전기 출력 = %.4f W/rotor, %.4f W total\n', ...
            bestPracticalCandidate.DeltaP_el_W, ...
            bestPracticalCandidate.DeltaP_el_W * RotorNo);
    end
end

fprintf('\n3) 형상별 최대 허용 두께\n');
for i = 1:height(maxThicknessTable)
    if maxThicknessTable.HasAcceptableCandidate(i)
        fprintf('   %-12s : %.1fmm 이하  (y/R=%.2f, z/R=%.2f, loss=%.3f%%)\n', ...
            maxThicknessTable.Shape(i), ...
            maxThicknessTable.MaxAllowableThickness_mm(i), ...
            maxThicknessTable.y_over_R(i), ...
            maxThicknessTable.z_over_R(i), ...
            maxThicknessTable.ThrustLoss_pct(i));
    else
        fprintf('   %-12s : 허용 손실률 이내 후보 없음\n', maxThicknessTable.Shape(i));
    end
end

%% 그래프
% Fig 1. 형상별 두께에 따른 최소 손실률
figure('Name','Thrust loss vs arm thickness', 'Position',[100 100 900 520]);
hold on;
for ss = 1:length(shapeName)
    shape      = shapeName(ss);
    minLossByD = nan(size(armThickness_mm_list));
    for dd = 1:length(armThickness_mm_list)
        sub = resultTable(resultTable.Shape == shape & ...
                          resultTable.ArmThickness_mm == armThickness_mm_list(dd), :);
        minLossByD(dd) = min(sub.ThrustLoss_pct);
    end
    plot(armThickness_mm_list, minLossByD, 'LineWidth', 1.6);
end
yline(targetLossPct, 'k--', 'Target loss');
hold off; grid on;
xlabel('Arm thickness or frontal height [mm]');
ylabel('Minimum thrust loss [%]');
title(sprintf('형상별 arm 두께에 따른 최소 추력 손실률 — %s', analysis_mode));
legend(shapeName, 'Location','northwest');

% Fig 2. 형상별 최대 허용 두께
figure('Name','Max allowable arm thickness', 'Position',[120 120 800 480]);
bar(categorical(maxThicknessTable.Shape), maxThicknessTable.MaxAllowableThickness_mm);
grid on;
ylabel('Maximum allowable arm thickness [mm]');
title(sprintf('형상별 최대 허용 arm 두께 (target loss %.1f%%) — %s', ...
    targetLossPct, analysis_mode));

% Fig 3. Offset 효과
figure('Name','Thrust loss vs y/R offset', 'Position',[140 140 900 520]);
hold on;
for ss = 1:length(shapeName)
    shape      = shapeName(ss);
    minLossByY = nan(size(yOverR_list));
    for yy = 1:length(yOverR_list)
        sub = resultTable(resultTable.Shape == shape & ...
                          resultTable.y_over_R == yOverR_list(yy), :);
        minLossByY(yy) = min(sub.ThrustLoss_pct);
    end
    plot(yOverR_list, minLossByY, 'LineWidth', 1.6);
end
yline(targetLossPct, 'k--', 'Target loss');
hold off; grid on;
xlabel('Arm offset y/R [-]');
ylabel('Minimum thrust loss [%]');
title('arm offset 증가에 따른 최소 추력 손실률');
legend(shapeName, 'Location','northeast');

% Fig 4. z/R 영향
figure('Name','Thrust loss vs z/R position', 'Position',[160 160 900 520]);
hold on;
for ss = 1:length(shapeName)
    shape      = shapeName(ss);
    minLossByZ = nan(size(zOverR_list));
    for zz = 1:length(zOverR_list)
        sub = resultTable(resultTable.Shape == shape & ...
                          resultTable.z_over_R == zOverR_list(zz), :);
        minLossByZ(zz) = min(sub.ThrustLoss_pct);
    end
    plot(zOverR_list, minLossByZ, 'LineWidth', 1.6);
end
yline(targetLossPct, 'k--', 'Target loss');
hold off; grid on;
xlabel('Vertical distance z/R [-]');
ylabel('Minimum thrust loss [%]');
title('arm 수직 위치 z/R에 따른 최소 추력 손실률');
legend(shapeName, 'Location','northwest');

%% =========================================================================
% 로컬 함수
function out = calcArmWakeResult( ...
    D, R, A_disk, T_N, P_mech, P_el, ...
    RPM, bladeNo, rho, St, ...
    shape, Cd, d_mm, yOverR, zOverR, wakeCrossMode, targetLossPct)

    d_m = d_mm / 1000;
    y   = yOverR * R;
    z   = zOverR * R;

    if y >= R
        L_wake = 0;
    else
        chordLength = 2 * sqrt(R^2 - y^2);
        switch string(wakeCrossMode)
            case "radius",   L_wake = chordLength / 2;
            case "diameter", L_wake = chordLength;
            otherwise
                error('wakeCrossMode는 "radius" 또는 "diameter"만 사용하세요.');
        end
    end

    v_i    = sqrt(T_N / (2 * rho * A_disk));
    k      = 1 + min(max(zOverR, 0), 1);   % z/R=0 → k=1, z/R>=1 → k=2
    v_wake = k * v_i;
    A_proj = d_m * L_wake;

    D_arm         = 0.5 * rho * v_wake^2 * Cd * A_proj;
    thrustLossPct = D_arm / T_N * 100;
    T_net         = T_N - D_arm;

    T_new      = T_N + D_arm;
    powerRatio = (T_new / T_N)^(3/2);

    P_new_mech      = P_mech * powerRatio;
    DeltaP_mech     = P_new_mech - P_mech;
    DeltaP_mech_pct = DeltaP_mech / P_mech * 100;

    if isnan(P_el)
        P_new_el = NaN; DeltaP_el = NaN; DeltaP_el_pct = NaN;
    else
        P_new_el      = P_el * powerRatio;
        DeltaP_el     = P_new_el - P_el;
        DeltaP_el_pct = DeltaP_el / P_el * 100;
    end

    f_rotor = RPM / 60;
    f_BPF   = bladeNo * f_rotor;
    f_shed  = NaN;
    if d_m > 0, f_shed = St * v_wake / d_m; end

    out.Shape           = string(shape);
    out.Cd              = Cd;
    out.ArmThickness_mm = d_mm;
    out.y_over_R        = yOverR;
    out.y_mm            = y * 1000;
    out.z_over_R        = zOverR;
    out.z_mm            = z * 1000;
    out.k               = k;
    out.L_wake_mm       = L_wake * 1000;
    out.A_proj_cm2      = A_proj * 1e4;
    out.Blockage_pct    = A_proj / A_disk * 100;
    out.v_i             = v_i;
    out.v_wake          = v_wake;
    out.D_arm_N         = D_arm;
    out.T_net_N         = T_net;
    out.ThrustLoss_pct  = thrustLossPct;
    out.P_new_mech_W    = P_new_mech;
    out.DeltaP_mech_W   = DeltaP_mech;
    out.DeltaP_mech_pct = DeltaP_mech_pct;
    out.P_new_el_W      = P_new_el;
    out.DeltaP_el_W     = DeltaP_el;
    out.DeltaP_el_pct   = DeltaP_el_pct;
    out.f_shed_Hz       = f_shed;
    out.f_rotor_Hz      = f_rotor;
    out.f_BPF_Hz        = f_BPF;
    out.Diff_to_rotor_Hz = abs(f_shed - f_rotor);
    out.Diff_to_BPF_Hz   = abs(f_shed - f_BPF);
    out.Acceptable       = thrustLossPct <= targetLossPct;
end