%% ------------------------------------------------------------------------
% Multirotor Sizing Methodology
% load_motorList.m (Excel version, assuming line-to-line resistance)
%% ------------------------------------------------------------------------

function motorList = load_motorList(voltage, prop_speedMax, prop_torqueMax, prop_speedHover, prop_torqueHover, spec_mass)
    motorList = {};
    
    % 엑셀 파일 읽기 (mfilename 기준 절대경로 → 어느 디렉토리에서도 동작)
    thisDir  = fileparts(mfilename('fullpath'));   % src/ 폴더
    filePath = fullfile(thisDir, '..', 'motor_list', 'Motor_list.xlsx');
    rawData = readcell(filePath);
    
    % 헤더 제외
    motorData = rawData(2:end, :);

    for ii = 1:size(motorData, 1)
        % 엑셀 열 매핑
        motor_name   = motorData{ii, 1};   % Name
        kV           = motorData{ii, 2};   % kV
        Rm           = motorData{ii, 3};   % Resistance (선간저항)
        current_max  = motorData{ii, 4};   % I_max
        mass         = motorData{ii, 5};   % Mass
        I0           = motorData{ii, 6};   % No-load current

        % 빈칸 또는 비정상 데이터 방지
        if isempty(motor_name) || isempty(kV) || isempty(Rm) || ...
           isempty(current_max) || isempty(mass) || isempty(I0)
            continue;
        end

        if ~isnumeric(kV) || ~isnumeric(Rm) || ~isnumeric(current_max) || ...
           ~isnumeric(mass) || ~isnumeric(I0)
            continue;
        end

        % 최대 전류 예외 처리
        if current_max < 1
            current_max = 2 * voltage;
        end

        % 철손 근사
        ironLoss = voltage * I0;

        % 프로펠러 요구 기계출력
        prop_powerMax   = prop_torqueMax   * prop_speedMax   / 60 * 2 * pi;
        prop_powerHover = prop_torqueHover * prop_speedHover / 60 * 2 * pi;

        % 판별식
        discMax   = voltage^2 - 4 * Rm * (ironLoss + prop_powerMax);
        discHover = voltage^2 - 4 * Rm * (ironLoss + prop_powerHover);

        if discMax < 0 || discHover < 0
            continue;
        end

        % 요구 전류 계산
        motor_currentMax   = (voltage - sqrt(discMax))   / (2 * Rm);
        motor_currentHover = (voltage - sqrt(discHover)) / (2 * Rm);

        % 필터링
        if isreal(motor_currentMax) && isreal(motor_currentHover) && ...
           motor_currentMax > 0 && motor_currentHover > 0 && ...
           motor_currentMax <= current_max && ...
           mass <= spec_mass && mass > 0 && ...
           0.8 * voltage * kV > prop_speedMax

            motor_powerMaxEl   = voltage * motor_currentMax;
            motor_effMax       = prop_powerMax / motor_powerMaxEl * 100;

            motor_powerHoverEl = voltage * motor_currentHover;
            motor_effHover     = prop_powerHover / motor_powerHoverEl * 100;

            % ID는 엑셀 행 번호 사용
            motor_id = ii;

            % 저장 구조:
            % ID, name, ILimit (A), mass (g), kV, Rm(line-line Ohm),
            % op_Imax (A), op_powerMaxEl (W), op_effMax (%),
            % op_IHover (A), op_powerHoverEl (W), op_effHover (%)
            motorList(end+1,:) = {motor_id, motor_name, current_max, mass, kV, Rm, ...
                                  motor_currentMax, motor_powerMaxEl, motor_effMax, ...
                                  motor_currentHover, motor_powerHoverEl, motor_effHover};
        end
    end
end