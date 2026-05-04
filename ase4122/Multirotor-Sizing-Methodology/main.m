%% ------------------------------------------------------------------------
% Multirotor Sizing Methodology
% with Flight Time Estimation
%
% M. Biczyski, R. Sehab, G.Krebs, J.F. Whidborne, P. Luk
%
% main.m - base file of the script realising input preparation, propeller
% and motor selection, battery simulation and results generation
%
% ── 실행 순서 ──────────────────────────────────────────────────────────────
%   [1안: 호버 최적화만]
%       main → aero_analysis → ground_effect_analysis → arm_wake_analysis
%
%   [2안: 기동성 최적화 포함]
%       main → forward_flight_analysis → ground_effect_analysis → arm_wake_analysis
%       ※ aero_analysis는 1안(호버 최적 프롭) 기준 FM/비추력 분석용
%
% ── 다운스트림 파일에 전달되는 주요 Workspace 변수 ──────────────────────
%   설계 파라미터:
%     RotorNo, SafetyFactor, OptimisationGoal, ThrustWeightRatio
%     BattCellNo, BattCellVoltage, BattCapacity, BattPeukertConstant
%     BattVoltageSagConstant, BattHourRating, mass_Motor_Est
%
%   추력 기준 ( 주의: 추정치 기반 / 실측 무게 반영 전):
%     thrustHover_Est  [g/rotor]  ← mass_Total_Est 기반
%     thrustMax_Est    [g/rotor]  ← thrustHover_Est × ThrustWeightRatio
%     ※ mass_Total(실측 보정치)은 모터·프롭 선정 후 재계산되지만
%        thrustHover_Est는 재계산하지 않음 (프롭 선정 기준 일관성 유지)
%
%   프롭 선정 결과 (1안 기준):
%     propList_considered  — 후보 프롭 전체 목록 (consideredNo × 6 cell)
%     propPerf             — 후보 프롭 성능 데이터 cell
%     operatingPoints      — 후보 프롭별 [hover/WOT/limit] 운용점
%     selectionCriterion   — 선정 점수 행렬 (inf = 탈락)
%     specThrust_criterion — 비추력 기준 점수 벡터
%     temp_propChosen_pos  — 선정 프롭 인덱스 (1안)
%     propSpecification    — {name, diam_in, pitch_in}
%     consideredNo         — 후보 프롭 수
%
%   모터/ESC 선정 결과 (1안 기준):
%     motorChosen          — 선정 모터 데이터 (1 × 12 cell)
%     motorSpecification   — {name, kV, RPM_max, torque_max, ...}
%     escSpecification     — 요구 ESC 전류 [A]
%     best_esc             — 선정 ESC 데이터 (1 × N cell)
%
%   질량 (주의: 두 가지 버전):
%     mass_Total_Est  [g]  ← 추정 부품 무게 기반 (프롭 선정에 사용)
%     mass_Total      [g]  ← 실제 선정 부품 무게 반영 (배터리 시뮬 및
%                            forward_flight_analysis에 사용)
%
%   배터리 시뮬:
%     time_hover, time_max  — 비행 가능 시간 벡터 [h]
%% ------------------------------------------------------------------------

close all; clear; clc;
format compact; format shortG;

thisFile = mfilename('fullpath');
thisDir  = fileparts(thisFile);
addpath(genpath(thisDir));

RPM2RAD = 2*pi/60;
RAD2RPM = 60/2/pi;

%% ── User parameters ──────────────────────────────────────────────────────
RotorNo           = 4;       % 로터 수
OptimisationGoal  = 'hover'; % 선정 기준
                             %   'hover'       : 호버 비추력 최대 (g/W)
                             %   'max'         : WOT 비추력 최대
                             %   'utilisation' : 프롭 속도 범위 활용 최대
ThrustWeightRatio = 2;       % 최대추력 / 이륙중량 비
                             %   2:최소 / 3:화물 / 4:감시 / 5+:에어로배틱 / 7+:레이싱
PropDiameter_Min  = 0;       % 최소 프롭 직경 [inch]
PropDiameter_Max  = 14;      % 최대 프롭 직경 [inch]
SafetyFactor      = 1.1;     % 안전 계수 [1-2]
AcceptedTypes     = {'MR' 'E-3' 'E-4'}; % 허용 프롭 시리즈
                             %   E:Electric / MR:Multi-Rotor / E-3:3-Blade / E-4:4-Blade

BattCellNo             = 4;    % 배터리 셀 수 [S]
BattCellVoltage        = 3.7;  % 셀 공칭 전압 [V]
BattCapacity           = 5200; % 배터리 용량 [mAh]
BattPeukertConstant    = 1.3;  % Peukert 상수 (LiPo 기준)
BattVoltageSagConstant = 0.5/0.8*BattCellNo; % 전압 강하 상수 (80% DoD 기준 셀당 0.5V)
BattHourRating         = 1;    % 배터리 기준 방전 시간 [h]

%% ── Mass data [g] ────────────────────────────────────────────────────────
% 고정 구조물
mass_Frame            = 222.5;
mass_FC               = 34.6;
mass_FC_GPS           = 7;
mass_FC_CurrentSensor = 15;
mass_Receiver         = 2;
mass_optical          = 1.5;
mass_Other_Est        = 20;   % 케이블, 스트랩 등

% 드라이브트레인 추정치 (프롭·모터 선정 전 초기 추정용)
% ※ mass_Motor_Est는 load_motorList 질량 필터 기준으로도 사용됨
mass_Motor_Est     = 93;
mass_ESC_Est       = 6.6;
mass_Propeller_Est = 18;

% 탑재물 및 배터리
% mass_Payload = 977;
mass_Payload = 500;
mass_Battery = 473;  % Lumenier 5200mAh 4S 35C

%% ── 총 무게 추정 (프롭·모터 선정 전 추정치) ───────────────────────────
% 이 값으로 thrustHover_Est를 계산하여 프롭/모터 선정 전 과정에 사용.
% 실제 선정 후 mass_Total(실측값)을 별도 재계산하지만,
% thrustHover_Est는 재계산하지 않음 → 선정 기준 일관성 유지.
mass_NoDrive_Est = mass_Frame + mass_FC + mass_FC_GPS + ...
    mass_FC_CurrentSensor + mass_Receiver + ...
    mass_Payload + mass_Other_Est + mass_optical;
mass_Total_Est = mass_NoDrive_Est + ...
    RotorNo*(mass_Motor_Est + mass_ESC_Est + mass_Propeller_Est) + mass_Battery;

fprintf('[질량 추정] mass_Total_Est = %.1f g  (프롭·모터 선정 기준)\n', mass_Total_Est);

%% ── 프롭 후보 로드 및 필터링 ────────────────────────────────────────────
% propList 컬럼: name, file, diameter(in), pitch(in), mass(g), speedLimit(RPM)
propList = load_propList();

idxSize = cellfun(@(x) x >= PropDiameter_Min && x <= PropDiameter_Max, propList(:,3));
idxType = cellfun(@(x) endsWith(x, AcceptedTypes), propList(:,1));
finalIdx = idxSize & idxType;

propList_considered = propList(finalIdx, :);
consideredNo = size(propList_considered, 1);
if consideredNo < 1
    error('ERROR! No matching propeller found!');
end
fprintf('[프롭 필터] 후보 %d개 선별 완료\n', consideredNo);

%% ── 프롭 성능 데이터 로드 ────────────────────────────────────────────────
% propPerf{ii} 컬럼: RPM, Thrust(g), Power(W), Torque(Nm), Cp, Ct
propPerf = {};
for ii = 1:consideredNo
    propPerf(ii) = {load_propPerf(propList_considered{ii,2}, false)};
end

%% ── 운용점 계산 ──────────────────────────────────────────────────────────
%   thrustHover_Est: mass_Total_Est 기반 (추정치)
%    실측 mass_Total 재계산 후에도 이 값은 유지 (프롭 선정 기준 일관성).
%    실측치와의 오차는 SafetyFactor(1.1)로 흡수.
thrustHover_Est = mass_Total_Est / RotorNo;  % [g/rotor]
thrustMax_Est   = thrustHover_Est * ThrustWeightRatio;

% operatingPoints{ii, 1} = [RPM, Thrust(g), Torque(Nm), Power(W)] @ hover
% operatingPoints{ii, 2} = [RPM, Thrust(g), Torque(Nm), Power(W)] @ WOT
% operatingPoints{ii, 3} = [RPM, Thrust(g), Torque(Nm), Power(W)] @ speedLimit
for ii = 1:consideredNo
    speedHover = interp1(propPerf{ii}(2:end,2), propPerf{ii}(2:end,1), thrustHover_Est);
    speedMax   = interp1(propPerf{ii}(2:end,2), propPerf{ii}(2:end,1), thrustMax_Est);
    speedLimit = propList_considered{ii,6};

    operatingPoints(ii,1) = {[speedHover, thrustHover_Est, ...
        interp1(propPerf{ii}(:,1), propPerf{ii}(:,4), speedHover), ...
        interp1(propPerf{ii}(:,1), propPerf{ii}(:,3), speedHover)]};
    operatingPoints(ii,2) = {[speedMax, thrustMax_Est, ...
        interp1(propPerf{ii}(:,1), propPerf{ii}(:,4), speedMax), ...
        interp1(propPerf{ii}(:,1), propPerf{ii}(:,3), speedMax)]};
    operatingPoints(ii,3) = {[speedLimit, ...
        interp1(propPerf{ii}(:,1), propPerf{ii}(:,2), speedLimit), ...
        interp1(propPerf{ii}(:,1), propPerf{ii}(:,4), speedLimit), ...
        interp1(propPerf{ii}(:,1), propPerf{ii}(:,3), speedLimit)]};
end

%% ── 프롭 선정 기준 계산 ─────────────────────────────────────────────────
for ii = 1:consideredNo
    switch OptimisationGoal
        case 'hover'
            selectionCriterion(ii,1) = (operatingPoints{ii,1}(1)*2*pi/60) * operatingPoints{ii,1}(3);
            selectionCriterion(ii,2) = operatingPoints{ii,1}(4);
        case 'max'
            selectionCriterion(ii,1) = (operatingPoints{ii,2}(1)*2*pi/60) * operatingPoints{ii,2}(3);
            selectionCriterion(ii,2) = operatingPoints{ii,2}(4);
        case 'utilisation'
            selectionCriterion(ii,1) = (operatingPoints{ii,3}(1)*2*pi/60)*operatingPoints{ii,3}(3) ...
                - (operatingPoints{ii,2}(1)*2*pi/60)*operatingPoints{ii,2}(3);
            selectionCriterion(ii,2) = operatingPoints{ii,3}(4) - operatingPoints{ii,2}(4);
        otherwise
            error('ERROR! Wrong optimisation criteria!');
    end
end

% methodError: 두 파워 측정 경로 간 차이 모니터링 (참고용)
%   col1 = ω × Torque 역산값,  col2 = APC 실측값
%   선정 기준으로는 사용하지 않으나, 데이터 품질 확인용으로 유지.
methodError(:,1) = abs(selectionCriterion(:,2) - selectionCriterion(:,1));
methodError(:,2) = abs(selectionCriterion(:,2) - selectionCriterion(:,1)) ./ abs(selectionCriterion(:,2));

% 속도 한계 초과 또는 보간 실패 탈락 처리
for ii = 1:consideredNo
    if operatingPoints{ii,3}(4) < operatingPoints{ii,2}(4) || isnan(operatingPoints{ii,2}(1))
        selectionCriterion(ii,:) = inf;
    end
end

% 비추력 기준 선정 (thrustHover_Est 모든 후보 동일 → 비추력최대 = 파워최소)
%
%   선정 기준: selectionCriterion col2 (APC 실측 파워, operatingPoints{ii,x}(4))
%   변경 이유:
%     col1 = ω × Torque  (보간된 토크로 역산한 파워)
%     col2 = APC .dat 실측 파워
%   두 값은 "보간 오차"가 아니라 서로 다른 측정 경로에서 온 값이므로
%   평균을 내면 물리적으로 애매한 값이 됨.
%   aero_analysis / arm_wake_analysis 모두 실측 파워(col2)를 사용하므로
%   선정 기준도 col2로 통일하여 파일 간 비추력 수치 일관성 확보.
%
%   참고: methodError는 두 측정 경로 간 차이를 모니터링하는 용도로 유지.
sc_P = selectionCriterion(:,2);          % APC 실측 파워 [W]
sc_P(isinf(sc_P) | isnan(sc_P)) = inf;
specThrust_criterion = thrustHover_Est ./ sc_P;
specThrust_criterion(isinf(sc_P)) = 0;
[~, temp_propChosen_pos] = max(specThrust_criterion);

if selectionCriterion(temp_propChosen_pos,2) == inf
    error('ERROR! No matching propeller found!');
end

%% ── 모터 로드 및 선정 ───────────────────────────────────────────────────
% load_motorList 입력: Vbatt, WOT_RPM, WOT_torque, hover_RPM, hover_torque(×SF), mass_limit
motorList = load_motorList( ...
    BattCellNo*BattCellVoltage, ...
    operatingPoints{temp_propChosen_pos,2}(1), ...
    operatingPoints{temp_propChosen_pos,2}(3), ...
    operatingPoints{temp_propChosen_pos,1}(1), ...
    operatingPoints{temp_propChosen_pos,1}(3)*SafetyFactor, ...
    mass_Motor_Est);

if size(motorList,1) < 1
    error('ERROR! No matching motor found!');
end

% motorList 컬럼:
%   1:ID  2:name  3:ILimit(A)  4:mass(g)  5:kV  6:Rm(Ohm)
%   7:op_Imax(A)  8:op_powerMaxEl(W)  9:op_effMax(%)
%   10:op_IHover(A)  11:op_powerHoverEl(W)  12:op_effHover(%)
switch OptimisationGoal
    case 'hover'
        [~, temp_motorChosen_pos] = min([motorList{:,11}]);
    case 'max'
        [~, temp_motorChosen_pos] = min([motorList{:,8}]);
    case 'utilisation'
        [~, temp_motorChosen_pos] = min(abs([motorList{:,3}]-[motorList{:,7}]));
    otherwise
        error('ERROR! Wrong optimisation criteria!');
end
motorChosen = motorList(temp_motorChosen_pos,:);

%% ── 드라이브 사양 확정 ──────────────────────────────────────────────────
% propSpecification  : {name, diam_in, pitch_in}
% motorSpecification : {name, kV, RPM_WOT, torque_WOT(×SF), power_WOT(×SF), effMax, powerMaxEl, Vnom}
% escSpecification   : 요구 ESC 전류 [A]
propSpecification  = propList_considered(temp_propChosen_pos, [1 3:4]);
motorSpecification = {motorChosen{2}, motorChosen{5}, ...
    operatingPoints{temp_propChosen_pos,2}(1), ...
    operatingPoints{temp_propChosen_pos,2}(3)*SafetyFactor, ...
    operatingPoints{temp_propChosen_pos,2}(4)*SafetyFactor, ...
    motorChosen{8}, motorChosen{9}, BattCellNo*BattCellVoltage};
escSpecification = motorChosen{7};

BattCapacity_Ah = BattCapacity / 1000;
minBattRating   = escSpecification * RotorNo * SafetyFactor / BattCapacity_Ah;
baterrySpecification = [BattCellNo, minBattRating, BattCapacity];

%% ── 총 무게 재계산 ( 실제 선정 부품 기반) ─────────────────────────────
% mass_Total: 실 선정 모터·프롭 무게를 반영한 최종 이륙 중량
% 사용처: 배터리 시뮬, forward_flight_analysis, ground_effect_analysis
% ※ thrustHover_Est는 이미 mass_Total_Est로 확정됨 → 갱신 안 함
mass_Propeller = propList_considered{temp_propChosen_pos,5};
mass_Motor     = motorChosen{4};
mass_ESC       = mass_ESC_Est;
mass_Total = mass_NoDrive_Est + ...
    RotorNo*(mass_Motor + mass_ESC + mass_Propeller) + mass_Battery;

fprintf('[질량 확정] mass_Total = %.1f g  (실제 선정 부품 반영)\n', mass_Total);
fprintf('            추정치 대비 오차: %+.1f g (%.1f%%)\n', ...
    mass_Total - mass_Total_Est, (mass_Total/mass_Total_Est - 1)*100);

%% ── 배터리 시뮬레이션 ───────────────────────────────────────────────────
voltage_hover(1) = BattCellNo * (BattCellVoltage + 0.5);
voltage_max(1)   = BattCellNo * (BattCellVoltage + 0.5);

current_hover(1) = motorChosen{11} * RotorNo / voltage_hover(1);
current_max(1)   = motorChosen{8}  * RotorNo / voltage_max(1);

capacity_hover(1) = (current_hover(1)^(1-BattPeukertConstant)) * ...
    (BattHourRating^(1-BattPeukertConstant)) * (BattCapacity_Ah^BattPeukertConstant);
capacity_max(1) = (current_max(1)^(1-BattPeukertConstant)) * ...
    (BattHourRating^(1-BattPeukertConstant)) * (BattCapacity_Ah^BattPeukertConstant);

timeStep = 1/3600;  % 1초 단위 [h]

ii = 1;
while voltage_hover(ii) > BattCellVoltage*BattCellNo && ii*timeStep < 2
    voltage_hover(ii+1) = voltage_hover(1) - ...
        (BattVoltageSagConstant/capacity_hover(1)) * (capacity_hover(1) - capacity_hover(ii));
    current_hover(ii+1) = motorChosen{11} * RotorNo / voltage_hover(ii+1);
    capacity_hover(ii+1) = (current_hover(ii+1)^(1-BattPeukertConstant)) * ...
        (BattHourRating^(1-BattPeukertConstant)) * (BattCapacity_Ah^BattPeukertConstant) ...
        - sum(current_hover(2:end) * timeStep);
    ii = ii + 1;
end
time_hover = (0:ii-1) * timeStep;

ii = 1;
while voltage_max(ii) > BattCellVoltage*BattCellNo && ii*timeStep < 2
    voltage_max(ii+1) = voltage_max(1) - ...
        (BattVoltageSagConstant/capacity_max(1)) * (capacity_max(1) - capacity_max(ii));
    current_max(ii+1) = motorChosen{8} * RotorNo / voltage_max(ii+1);
    capacity_max(ii+1) = (current_max(ii+1)^(1-BattPeukertConstant)) * ...
        (BattHourRating^(1-BattPeukertConstant)) * (BattCapacity_Ah^BattPeukertConstant) ...
        - sum(current_max(2:end) * timeStep);
    ii = ii + 1;
end
time_max = (0:ii-1) * timeStep;

%% ── ESC 선정 ────────────────────────────────────────────────────────────
esc_List = load_escList();

% esc_margin: ESC 선정용 안전 여유 (forward_flight의 1.2와 동일하게 통일)
esc_margin      = 1.2;
req_esc_current = ceil(escSpecification) * esc_margin;

best_esc = {};
min_esc_mass = inf;
for i = 1:size(esc_List, 1)
    if esc_List{i,3} >= req_esc_current && esc_List{i,4} < min_esc_mass
        min_esc_mass = esc_List{i,4};
        best_esc = esc_List(i,:);
    end
end

%% ── 결과 출력 ───────────────────────────────────────────────────────────
disp(['For a ' num2str(RotorNo) '-rotor drone with estimated AUM of ' ...
    num2str(round(mass_Total_Est)) ' g (calculated TOM of ' num2str(round(mass_Total)) ' g):']);

switch OptimisationGoal
    case 'hover'
        textOptimisation = ['the highest specific thrust of ' ...
            num2str(round(operatingPoints{temp_propChosen_pos,1}(2)/motorChosen{11}*100)/100) ...
            ' gf/W per motor at hover.'];
    case 'max'
        textOptimisation = ['the highest specific thrust of ' ...
            num2str(round(operatingPoints{temp_propChosen_pos,2}(2)/motorChosen{8}*100)/100) ...
            ' gf/W per motor at WOT.'];
    case 'utilisation'
        textOptimisation = 'maximum usable power range of propeller';
    otherwise
        error('ERROR! Wrong optimisation criteria!');
end

disp(['APC ' propSpecification{1} ' propeller should be chosen for ' textOptimisation]);
disp([motorSpecification{1} ' (' num2str(round(motorSpecification{2}/10)*10) ...
    ' KV) motor should be selected with ' ...
    num2str(round(motorSpecification{4}*100)/100) ' Nm torque at maximum speed of ' ...
    num2str(round(motorSpecification{3}/100)*100) ' RPM.']);
disp(['One motor uses ' num2str(round(motorChosen{11})) ...
    ' W of electrical power at hover and ' num2str(round(motorChosen{8})) ...
    ' W of electrical power at WOT.']);
disp(['The drive should be controlled by a ' num2str(ceil(escSpecification)) ' A ESC per motor.']);
disp(['The whole system should be powered by a ' num2str(baterrySpecification(1)) 'S ' ...
    num2str(ceil(baterrySpecification(2))) 'C LiPo battery of ' ...
    num2str(baterrySpecification(3)) ' mAh.']);
disp('---------');
disp(['Hovering flight requires ' ...
    num2str(round(RotorNo*operatingPoints{temp_propChosen_pos,1}(4))) ...
    ' W of mechanical power (' ...
    num2str(round(operatingPoints{temp_propChosen_pos,1}(3)*100)/100) ' Nm at ' ...
    num2str(round(operatingPoints{temp_propChosen_pos,1}(1)/100)*100) ...
    ' RPM) to achieve ' ...
    num2str(round(operatingPoints{temp_propChosen_pos,1}(2)*RotorNo)) ' gf of total thrust.']);
disp(['WOT flight requires ' ...
    num2str(round(RotorNo*operatingPoints{temp_propChosen_pos,2}(4))) ...
    ' W of mechanical power (' ...
    num2str(round(operatingPoints{temp_propChosen_pos,2}(3)*100)/100) ' Nm at ' ...
    num2str(round(operatingPoints{temp_propChosen_pos,2}(1)/100)*100) ...
    ' RPM) to achieve ' ...
    num2str(round(operatingPoints{temp_propChosen_pos,2}(2)*RotorNo)) ' gf of total thrust.']);
disp(['This configuration should achieve around ' num2str(round(time_hover(end)*60)) ...
    ' min of hover and around ' num2str(round(time_max(end)*60)) ' min of flight at WOT.']);

plot_propPerf;
plot_motorPerf;

if isempty(best_esc)
    disp('조건을 만족하는 ESC가 리스트에 없습니다.');
else
    fprintf('선정된 ESC: %s (허용 전류: %dA, 무게: %.1fg)\n', ...
        best_esc{2}, best_esc{3}, best_esc{4});
end