%% ------------------------------------------------------------------------
% Multirotor Sizing Methodology
% plot_motorPerf.m (Excel version, line-to-line resistance assumed)
%
% main.m 또는 example 파일을 먼저 실행한 뒤 사용
% addaxis / addaxisplot 함수 필요
%% ------------------------------------------------------------------------

% 엑셀 파일 읽기
rawData = readcell('motor_list/Motor_list.xlsx');

% 헤더 제외 데이터
motorData = rawData(2:end,:);

% motorChosen{1} = load_motorList에서 저장한 motor_id = 엑셀 데이터 행 번호
row_id = motorChosen{1};

% 해당 모터의 I0 읽기 (6열)
I0 = motorData{row_id, 6};

% 최대 전류
current_maximum = motorChosen{3};
if current_maximum < 1
    current_maximum = 2 * BattCellNo * BattCellVoltage;
end

current = 1:current_maximum;

% load_motorList와 동일한 기준 사용
% motorChosen 구조:
% {1}=ID, {2}=name, {3}=ILimit, {4}=mass, {5}=kV, {6}=Rm(line-line),
% {7}=op_Imax, {8}=op_powerMaxEl, {9}=op_effMax,
% {10}=op_IHover, {11}=op_powerHoverEl, {12}=op_effHover

Rm = motorChosen{6};   % 선간저항
kV = motorChosen{5};   % kV

% 손실 계산
coppLoss = Rm * current.^2;
ironLoss = BattCellNo * BattCellVoltage * I0;
loss = coppLoss + ironLoss;

% 성능 계산
speed = (BattCellNo * BattCellVoltage - coppLoss ./ current) * kV;
powerMax = BattCellNo * BattCellVoltage * current;
power = powerMax - loss;
efficiency = power ./ powerMax;

%% Motor performance characteristics
figure;
plot(current, speed);
ylim([0 ceil(max(speed) * 1.1 / 1000) * 1000]);

ax = gca;
ax.YColor = [0, 0.4470, 0.7410];

scaler = ceil(max(power) * 1.1 / 100) * 100;
addaxis(current, power, [0 scaler]);
addaxisplot(current, powerMax, 2, 'k-.');
addaxis(current(2:end), efficiency(2:end) * 100, [40 100], ...
        'Color', [0.4660, 0.6740, 0.1880]);

% 운용점 마킹
addaxisplot(motorChosen{7}, ...
    operatingPoints{temp_propChosen_pos,2}(3) * ...
    operatingPoints{temp_propChosen_pos,2}(1) * RPM2RAD, ...
    2, '.', 'MarkerSize', 20);

addaxisplot(motorChosen{10}, ...
    operatingPoints{temp_propChosen_pos,1}(3) * SafetyFactor * ...
    operatingPoints{temp_propChosen_pos,1}(1) * RPM2RAD, ...
    2, '.', 'MarkerSize', 20);

addaxisplot(motorChosen{7}, motorChosen{9}, 3, '.', 'MarkerSize', 20);
addaxisplot(motorChosen{10}, motorChosen{12}, 3, '.', 'MarkerSize', 20);

grid on;
grid minor;
xlim([0 ceil(current_maximum / 10) * 10]);
xlabel('Current (A)');
title([char(motorChosen{2}) ' (' num2str(round(motorChosen{5} / 10) * 10) ' kV)']);
legend('Speed (RPM)', 'Mech. power (W)', 'El. Power (W)', 'Efficiency (%)', ...
       'Location', 'southeast');