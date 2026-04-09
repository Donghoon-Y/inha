%% ------------------------------------------------------------------------
% Multirotor Sizing Methodology
% with Flight Time Estimation
%
% M. Biczyski, R. Sehab, G.Krebs, J.F. Whidborne, P. Luk
%
% main.m - base file of the script realising input preparation, propeller 
% and motor selection, battery simulation and results generation
%% ------------------------------------------------------------------------

close all; clear; clc;
format compact; format shortG;

thisFile = mfilename('fullpath');
thisDir  = fileparts(thisFile);
addpath(genpath(thisDir)); 

RPM2RAD = 2*pi/60;
RAD2RPM = 60/2/pi;
%% User parameters
RotorNo = 4; % Number of rotors
OptimisationGoal = 'utilisation'; % selection criteria
                    % hover - best specific thrust (g/W) at hover
                    % max - best specific thrust (g/W) at 100% throttle
                    % utilisation - maximum usable power range of propeller
ThrustWeightRatio = 2; % estimator for maximum paerformance
                    % 2 - minimum
                    % 3 - payload transport
                    % 4 - survaillence
                    % 5+ - aerobatics / hi-speed video
                    % 7+ - racing
PropDiameter_Min = 0; % inch, min. propeller diameter
PropDiameter_Max = 14; % inch, max. propeller diameter
SafetyFactor = 1.1; % [1-2], arbitrary safety parameter
AcceptedTypes = {'MR' 'E-3' 'E-4'}; % preferred propeller series
                  % E	Electric
                  % F	Folding Blade (electric only)
                  % MR	Multi-Rotor (electric only)
                  % SF	Slow Fly (electric only)
                  % E-3	3-Blade
                  % E-4	4-Blade

BattCellNo = 4; %S 1P, battery cell count
BattCellVoltage = 3.7; % V per cell, battery cell voltage
BattCapacity = 5200; % mAh, battery capacity
BattPeukertConstant = 1.3; % for LiPo, Peukert's constant for selected type of battery
BattVoltageSagConstant = 0.5/0.8*BattCellNo; % 0.5V decrease per cell in resting volatage for 80% DoD
BattHourRating = 1; % h

%% Mass data [g] %% quadrotor에 무게로써 고려될 수 있는 것들
mass_Frame = 222.5; 
mass_FC = 34.6; 
mass_FC_GPS = 7;
mass_FC_CurrentSensor = 15;
mass_Receiver = 2; 
mass_Motor_Est = 94;
mass_ESC_Est = 6.6; 
mass_Propeller_Est = 18; 
mass_optical = 1.5;

% mass_Payload = 977;
mass_Payload = 500;
mass_Battery = 473; % Lumenier 5200mAh 4s 35c
mass_Other_Est = 20; % cabling, straps, standoffs, etc.

%% 무게들을 합산했을 때 이거를 운용할 때와 운용하지 않을 때로 나눔. 
mass_NoDrive_Est = mass_Frame + mass_FC + mass_FC_GPS + mass_FC_CurrentSensor + mass_Receiver + mass_Payload + mass_Other_Est + mass_optical;
mass_Total_Est = mass_NoDrive_Est + RotorNo*(mass_Motor_Est + mass_ESC_Est + mass_Propeller_Est) + mass_Battery;

%% Filter propeller set
%% propList = name, file, diameter (in), pitch (in), mass (g), speedLimit (RPM) -> 이거 자체가 데이터 구조.
propList = load_propList(); % loading propeller set

%% Filter propeller set
% 직경 조건: 설정한 최소(Min) 및 최대(Max) 직경 범위 내에 있는지 확인 [cite: 209, 215]
idxSize = cellfun(@(x) x >= PropDiameter_Min && x <= PropDiameter_Max, propList(:,3));

% 타입 조건: AcceptedTypes('MR', 'E-3' 등)로 끝나는 이름인지 확인 [cite: 211, 213]
idxType = cellfun(@(x) endsWith(x, AcceptedTypes), propList(:,1));

% 두 조건을 모두 만족(AND 연산)하는 행만 선택합니다.
% 이제 질량 조건(idxMass)은 고려하지 않습니다.
finalIdx = idxSize & idxType;

% 최종 후보 목록 결정
propList_considered = propList(finalIdx, :);
consideredNo = size(propList_considered, 1); % 최종 후보 개수 산출 [cite: 215]
if consideredNo < 1
    error('ERROR! No matching propeller found!');
end

%% Load propeller performance
% propPerf = RPM, Thrust (g), Power (W), Torque (Nm), Cp, Ct
% operatingPoints = {hover, max, limit}
%                   [speed, thrust, torque, power]
%% propPerf를 통해서 정적인 성능을 해석함.
propPerf = {};
for ii = 1:consideredNo
    % TRUE/FALSE for plot 그래프를 그릴 것인지 말 것인지에 대한 이야기
    propPerf(ii) = {load_propPerf(propList_considered{ii,2}, false)}; % loading propeller static performance data
end

%% Calculate operating points
thrustHover_Est = mass_Total_Est/RotorNo; % calcculate thrust required for hover
thrustMax_Est = thrustHover_Est*ThrustWeightRatio; % calculate estimated thrust at WOT
%% 우리가 프로펠러를 선별한 것들을 토대로 이제 그것들을 하나하나 계산해줌. 
for ii = 1:consideredNo
    % 추력, RPM에 대해서 그래프를 만들었을 때 추력이 thrustHover_Est이면 어느 정도의 RPM이 필요할 것인가.
    speedHover = interp1(propPerf{ii}(2:end,2), propPerf{ii}(2:end,1), thrustHover_Est); % obtaining propeller speed at hover from required thrust for hover
    % 추력, RPM에 대해서 그래프를 만들었을 때 추력이 thrustMax_Est이면 어느 정도의 RPM이 필요할 것인가.
    speedMax = interp1(propPerf{ii}(2:end,2), propPerf{ii}(2:end,1), thrustMax_Est); % obtaining propeller speed at WOT from estimated thrust at WOT 
    % 프롭의 한계 속도 
    speedLimit = propList_considered{ii,6}; % obtaining propeller's limiting speed specified by the manufacturer
    % speedHover: 호버 RPM / thrustHover_Est: 호버 추력 / interp1(..., torque, speedHover) : 그 RPM에서의 토크
    % interp1(..., power, speedHover) : 그 RPM에서의 파워
    operatingPoints(ii,1) = {[speedHover thrustHover_Est interp1(propPerf{ii}(:,1), propPerf{ii}(:,4), speedHover) interp1(propPerf{ii}(:,1), propPerf{ii}(:,3), speedHover)]}; % obtaining hover operating point
    operatingPoints(ii,2) = {[speedMax thrustMax_Est interp1(propPerf{ii}(:,1), propPerf{ii}(:,4), speedMax) interp1(propPerf{ii}(:,1), propPerf{ii}(:,3), speedMax)]}; % obtaining WOT operating point
    operatingPoints(ii,3) = {[speedLimit interp1(propPerf{ii}(:,1), propPerf{ii}(:,2), speedLimit) interp1(propPerf{ii}(:,1), propPerf{ii}(:,4), speedLimit)...
        interp1(propPerf{ii}(:,1), propPerf{ii}(:,3), speedLimit)]}; % obtaining speed limit operating point
end
% 즉 hover 운용점, max 요구 추력 운용점, speed limit 운용점에 대해서 clustering 함.
%% Select propeller
for ii = 1:consideredNo
    switch OptimisationGoal % selection of approperiate criteria based on user's choice
        case 'hover'
            selectionCriterion(ii,1) = (operatingPoints{ii,1}(1)*2*pi/60)*operatingPoints{ii,1}(3); % Power = 각속도* 토크
            selectionCriterion(ii,2) = operatingPoints{ii,1}(4);  % power at hover                  % 데이터에 있는 토크
        case 'max'
            selectionCriterion(ii,1) = (operatingPoints{ii,2}(1)*2*pi/60)*operatingPoints{ii,2}(3);
            selectionCriterion(ii,2) = operatingPoints{ii,2}(4); % power at WOT
        case 'utilisation'  % 최대 요구 추력 상태와 제조사 제한 RPM 상태 사이에 여유가 얼마나 있느냐
            selectionCriterion(ii,1) = (operatingPoints{ii,3}(1)*2*pi/60)*operatingPoints{ii,3}(3) - (operatingPoints{ii,2}(1)*2*pi/60)*operatingPoints{ii,2}(3);
            selectionCriterion(ii,2) = operatingPoints{ii,3}(4) - operatingPoints{ii,2}(4); % best usage of propeller's speed range
        otherwise
            error('ERROR! Wrong optimisation criteria!');
    end
end
%% 보간 때문에 생기는 절대 오차와 상대 오차
methodError(:,1) = abs(selectionCriterion(:,2) - selectionCriterion(:,1)); % absolute error between power and the product of speed and torque due to interpolation
methodError(:,2) = abs(selectionCriterion(:,2) - selectionCriterion(:,1))./abs(selectionCriterion(:,2)); % relative interpolation error
for ii = 1:consideredNo
    if operatingPoints{ii,3}(4) < operatingPoints{ii,2}(4) || isnan(operatingPoints{ii,2}(1))
        selectionCriterion(ii,:) = inf; % rejecting propellers with numerical errors and with WOT speed over limit speed
    end
end
% 가장 power가 적은 프로펠러 데이터를 가져옴.
[~, temp_propChosen_pos] = min(mean(selectionCriterion,2)); % selection of best propeller for the application

if selectionCriterion(temp_propChosen_pos,2) == inf
    error('ERROR! No matching propeller found!');
end

%% Load & filter motor data
% motorList = ID, name, ILimit (A), mass (g), kV, Rm (Ohm),
            % op_Imax (A), op_powerMaxEl(W), op_effMax (%), op_IHover (A), op_powerHoverEl (W), op_effHover (%)
            
motorList = load_motorList(BattCellNo*BattCellVoltage, operatingPoints{temp_propChosen_pos,2}(1), operatingPoints{temp_propChosen_pos,2}(3),...
    operatingPoints{temp_propChosen_pos,1}(1), operatingPoints{temp_propChosen_pos,1}(3)*SafetyFactor,...
    mass_Motor_Est); % loading motor set with operating points

if size(motorList,1) < 1
    error('ERROR! No matching motor found!');
end

%% Select motor
switch OptimisationGoal % selection of approperiate criteria based on user's choice
    case 'hover'
        [~, temp_motorChosen_pos] = min([motorList{:,11}]); % power at hover
    case 'max'
        [~, temp_motorChosen_pos] = min([motorList{:,8}]); % power at WOT
    case 'utilisation'
        [~, temp_motorChosen_pos] = min(abs([motorList{:,3}]-[motorList{:,7}]));  % best usage of motor's current range
    otherwise
        error('ERROR! Wrong optimisation criteria!');
end

motorChosen = motorList(temp_motorChosen_pos,:); % selection of best motor for the application

%% Determine drive specification
% propSpecification = name, diameter (in), pitch (in)
% motorSpecification = name, speedMax (RPM), torqueMax (Nm), powerMax (W), powerMaxEl (W), EfficiencyMax(%), voltageNominal (V)
% escSpecification = currentMax (A)
% baterrySpecification = NoCells, C-rating, minCapacity (mAh)

propSpecification = propList_considered(temp_propChosen_pos,[1 3:4]);
motorSpecification = {motorChosen{2}, motorChosen{5}, operatingPoints{temp_propChosen_pos,2}(1), operatingPoints{temp_propChosen_pos,2}(3)*SafetyFactor,...
    operatingPoints{temp_propChosen_pos,2}(4)*SafetyFactor, motorChosen{8}, motorChosen{9}, BattCellNo*BattCellVoltage};
escSpecification = motorChosen{7};

BattCapacity_Ah = BattCapacity/1000;
minBattRating = escSpecification*RotorNo*SafetyFactor/BattCapacity_Ah; % calculate min. battery C-rating required to supply enough current to motors
baterrySpecification = [BattCellNo, minBattRating, BattCapacity];

mass_Propeller = propList_considered{temp_propChosen_pos,5};
mass_Motor = motorChosen{4};
mass_ESC = mass_ESC_Est;
mass_Total = mass_NoDrive_Est + RotorNo*(mass_Motor + mass_ESC + mass_Propeller) + mass_Battery; % recalculate total mass of multirotor using real component weights

%% Calculate initioal battery state
voltage_hover(1) = (BattCellNo*(BattCellVoltage+0.5)); % 4.2 V per cell times number of cells
voltage_max(1) = (BattCellNo*(BattCellVoltage+0.5));
current_hover(1) = motorChosen{11}*RotorNo/voltage_hover(1); % calculate total current at hover
current_max(1) = motorChosen{8}*RotorNo/voltage_max(1); % calculate total current at WOT
capacity_hover(1) = (current_hover(1)^(1-BattPeukertConstant))*(BattHourRating^(1-BattPeukertConstant))*(BattCapacity_Ah^BattPeukertConstant); % from modified Peukert's equation calculate available capacity at hover
capacity_max(1) = (current_max(1)^(1-BattPeukertConstant))*(BattHourRating^(1-BattPeukertConstant))*(BattCapacity_Ah^BattPeukertConstant); % from modified Peukert's equation calculate available capacity at WOT

%% Calculate next flight iterations
timeStep = 1/60/60; % set timestep as 1 s
ii = 1;
while voltage_hover(ii) > BattCellVoltage*BattCellNo && ii*timeStep < 2
    voltage_hover(ii+1) = voltage_hover(1) - (BattVoltageSagConstant/capacity_hover(1))*(capacity_hover(1) - capacity_hover(ii)); % calculate instantaneus voltage including voltage sag
    current_hover(ii+1) = motorChosen{11}*RotorNo/voltage_hover(ii+1); % calculate instantaneus current based on required power for hover
    capacity_hover(ii+1) = (current_hover(ii+1)^(1-BattPeukertConstant))*(BattHourRating^(1-BattPeukertConstant))*(BattCapacity_Ah^BattPeukertConstant) - sum(current_hover(2:end)*timeStep); % calculate remaining available capacity according to Paeukert's effect
    ii = ii+1;
end
time_hover = (0:ii-1)*timeStep; % calculate time spent in hover

ii = 1;
while voltage_max(ii) > BattCellVoltage*BattCellNo && ii*timeStep < 2
    voltage_max(ii+1) = voltage_max(1) - (BattVoltageSagConstant/capacity_max(1))*(capacity_max(1) - capacity_max(ii)); % calculate instantaneus voltage including voltage sag
    current_max(ii+1) = motorChosen{8}*RotorNo/voltage_max(ii+1); % calculate instantaneus current based on estimated power at WOT
    capacity_max(ii+1) = (current_max(ii+1)^(1-BattPeukertConstant))*(BattHourRating^(1-BattPeukertConstant))*(BattCapacity_Ah^BattPeukertConstant) - sum(current_max(2:end)*timeStep); % calculate remaining available capacity according to Paeukert's effect
    ii = ii+1;
end
time_max = (0:ii-1)*timeStep; % calculate time spent at WOT

%% Display results and plot characteristics
disp(['For a ' num2str(RotorNo) '-rotor drone with estimated AUM of ' num2str(round(mass_Total_Est)) ' g (calculated TOM of ' num2str(round(mass_Total)) ' g):']);

switch OptimisationGoal
    case 'hover'
        textOptimisation = ['the highest specific thrust of ' num2str(round(operatingPoints{temp_propChosen_pos,1}(2)/motorChosen{11}*100)/100)  ' gf/W per motor at hover.'];
    case 'max'
        textOptimisation = ['the highest specific thrust of ' num2str(round(operatingPoints{temp_propChosen_pos,2}(2)/motorChosen{8}*100)/100)  ' gf/W per motor at WOT.'];
    case 'utilisation'
        textOptimisation = 'maximum usable power range of propeller';
    otherwise
        error('ERROR! Wrong optimisation criteria!');
end
        
disp(['APC ' propSpecification{1} ' propeller should be chosen for ' textOptimisation]);
disp([motorSpecification{1} ' (' num2str(round(motorSpecification{2}/10)*10) ' KV) motor should be selected with '...
    num2str(round(motorSpecification{4}*100)/100) ' Nm torque at maximum speed of ' num2str(round(motorSpecification{3}/100)*100) ' RPM.']);
disp(['One motor uses ' num2str(round(motorChosen{11})) ' W of electrical power at hover and ' num2str(round(motorChosen{8})) ' W of electrical power at WOT.']);
disp(['The drive should be controlled by a ' num2str(ceil(escSpecification)) ' A ESC per motor.']);
disp(['The whole system should be powered by a ' num2str(baterrySpecification(1)) 'S ' num2str(ceil(baterrySpecification(2))) 'C LiPo battery of '...
    num2str(baterrySpecification(3)) ' mAh.']);
disp('---------');
disp(['Hovering flight requires ' num2str(round(RotorNo*operatingPoints{temp_propChosen_pos,1}(4))) ' W of mechanical power (' num2str(round(operatingPoints{temp_propChosen_pos,1}(3)*100)/100)...
    ' Nm at ' num2str(round(operatingPoints{temp_propChosen_pos,1}(1)/100)*100) ' RPM) to achieve ' num2str(round(operatingPoints{temp_propChosen_pos,1}(2)*RotorNo)) ' gf of total thrust.']);
disp(['WOT flight requires ' num2str(round(RotorNo*operatingPoints{temp_propChosen_pos,2}(4))) ' W of mechanical power (' num2str(round(operatingPoints{temp_propChosen_pos,2}(3)*100)/100)...
    ' Nm at ' num2str(round(operatingPoints{temp_propChosen_pos,2}(1)/100)*100) ' RPM) to achieve ' num2str(round(operatingPoints{temp_propChosen_pos,2}(2)*RotorNo)) ' gf of total thrust.']);
disp(['This configuration should achieve around ' num2str(round(time_hover(end)*60)) ' min of hover and around ' num2str(round(time_max(end)*60)) ' min of flight at WOT.']);

plot_propPerf; % plot propeller performance & battery simulation results 
plot_motorPerf; % plot motor performance
