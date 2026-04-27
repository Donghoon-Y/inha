%% ------------------------------------------------------------------------
% Multirotor Sizing Methodology
% load_motorList.m (Excel version, assuming line-to-line resistance)
%
% [필터 정책]
%   하드 탈락 (물리적 불가 / 안전):
%     1. disc < 0          → 해당 전압으로 필요 토크/파워 자체가 불가
%     2. I_req > I_max     → 모터 정격 초과, 손상 위험
%     3. kV_margin < 0.7   → 풀스로틀에서도 필요 RPM의 70%도 못 미침
%
%   소프트 페널티 (점수에 반영, 탈락 없음):
%     4. mass > spec_mass  → 설계 추정값 초과 (초과 비율만큼 페널티)
%     5. kV_margin 0.7~1.0 → RPM 마진 빡빡 (부족할수록 페널티)
%
%   반환: 점수 오름차순 정렬 (낮을수록 좋음) → 항상 후보 반환
%% ------------------------------------------------------------------------

function motorList = load_motorList(voltage, prop_speedMax, prop_torqueMax, prop_speedHover, prop_torqueHover, spec_mass)
    motorList = {};

    % 엑셀 파일 읽기 (mfilename 기준 절대경로)
    thisDir  = fileparts(mfilename('fullpath'));
    filePath = fullfile(thisDir, '..', 'motor_list', 'Motor_list.xlsx');
    rawData  = readcell(filePath);
    motorData = rawData(2:end, :);

    tempList  = {};
    scoreList = [];

    for ii = 1:size(motorData, 1)
        motor_name   = motorData{ii, 1};
        kV           = motorData{ii, 2};
        Rm           = motorData{ii, 3};
        current_max  = motorData{ii, 4};
        mass         = motorData{ii, 5};
        I0           = motorData{ii, 6};

        % 빈칸 / 비숫자 스킵
        if isempty(motor_name) || isempty(kV) || isempty(Rm) || ...
           isempty(current_max) || isempty(mass) || isempty(I0)
            continue;
        end
        if ~isnumeric(kV) || ~isnumeric(Rm) || ~isnumeric(current_max) || ...
           ~isnumeric(mass) || ~isnumeric(I0)
            continue;
        end
        if current_max < 1
            current_max = 2 * voltage;
        end

        % 기계출력 계산
        ironLoss        = voltage * I0;
        prop_powerMax   = prop_torqueMax   * prop_speedMax   / 60 * 2 * pi;
        prop_powerHover = prop_torqueHover * prop_speedHover / 60 * 2 * pi;

        % ── 하드 탈락 1: 판별식 (물리적으로 해 없음) ─────────────────────
        discMax   = voltage^2 - 4 * Rm * (ironLoss + prop_powerMax);
        discHover = voltage^2 - 4 * Rm * (ironLoss + prop_powerHover);
        if discMax < 0 || discHover < 0
            continue;
        end

        motor_currentMax   = (voltage - sqrt(discMax))   / (2 * Rm);
        motor_currentHover = (voltage - sqrt(discHover)) / (2 * Rm);

        if ~isreal(motor_currentMax) || ~isreal(motor_currentHover) || ...
           motor_currentMax <= 0    || motor_currentHover <= 0
            continue;
        end

        % ── 하드 탈락 2: I_max 초과 (모터 손상) ──────────────────────────
        if motor_currentMax > current_max
            continue;
        end

        % ── 하드 탈락 3: kV 마진 < 0.7 (풀스로틀에서도 RPM 70% 미달) ────
        % kV_margin = (0.8 * V * kV) / speedMax
        % 0.8: 전압 새그 마진, 결과가 1.0 이상이면 충분한 여유
        kv_margin = (0.8 * voltage * kV) / prop_speedMax;
        if kv_margin < 0.7
            continue;
        end

        % ── 소프트 점수 계산 (기준: 호버 소비전력, 낮을수록 좋음) ─────────
        motor_powerMaxEl   = voltage * motor_currentMax;
        motor_powerHoverEl = voltage * motor_currentHover;
        motor_effMax       = prop_powerMax   / motor_powerMaxEl * 100;
        motor_effHover     = prop_powerHover / motor_powerHoverEl * 100;

        score = motor_powerHoverEl;

        % 소프트 페널티 4: 질량 초과
        if mass > spec_mass
            score = score * (1 + (mass - spec_mass) / spec_mass);
        end

        % 소프트 페널티 5: kV 마진 0.7~1.0 구간 (빡빡할수록 페널티)
        if kv_margin < 1.0
            score = score * (1 + (1.0 - kv_margin) * 2);
        end

        % ── 후보 저장 ────────────────────────────────────────────────────
        tempList(end+1,:) = {ii, motor_name, current_max, mass, kV, Rm, ...
                             motor_currentMax, motor_powerMaxEl, motor_effMax, ...
                             motor_currentHover, motor_powerHoverEl, motor_effHover};
        scoreList(end+1)  = score;
    end

    % 점수 오름차순 정렬
    if isempty(tempList)
        motorList = {};
        return;
    end
    [~, sortIdx] = sort(scoreList);
    motorList = tempList(sortIdx, :);
end