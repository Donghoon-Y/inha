%% =========================================================================
% ground_effect_analysis.m
%
% 실행 순서:
%
%   [호버 최적 프롭 기준 지면효과 분석]
%       main  → aero_analysis → arm_wake_analysis  →planar_analysis → ground_effect_analysis
%
%   [전진비행 최적 프롭 기준 지면효과 분석]
%       main  → forward_flight_analysis → arm_wake_analysis  →planar_analysis → ground_effect_analysis
%
% 목적:
%   main.m 또는 forward_flight_analysis.m 실행 후 workspace에 남아있는
%   최적 프롭 정보와 공력해석을 통해 정해진 arm 길이 기준 지면효과와 랜딩기어 높이를 분석한다.
%
% 
% =========================================================================

%% -------------------------------------------------------------------------
% [사전 확인] main.m 실행 여부
% -------------------------------------------------------------------------
if ~exist('propSpecification','var') || ~exist('mass_Total','var') || ~exist('RotorNo','var')
    error('main.m을 먼저 실행해야 합니다. 필요한 변수: propSpecification, mass_Total, RotorNo');
end


%% -------------------------------------------------------------------------
% [분석 대상 프롭 자동 선택]
% -------------------------------------------------------------------------
% 원리:
%   1. forward_flight_analysis.m 실행 후에는 prop_ff 변수가 생성됨.
%      이 경우 전진비행 최적 프롭을 지면효과 분석 대상으로 사용.
%
%   2. prop_ff가 없으면 main.m의 propSpecification을 사용.
%      이 경우 호버 최적 프롭을 지면효과 분석 대상으로 사용.

if exist('prop_ff','var') && ~isempty(prop_ff)
    analysis_mode   = 'Forward-flight optimum propeller';
    prop_name_ge    = prop_ff{1};
    prop_diam_in_ge = prop_ff{3};
    prop_pitch_in_ge = prop_ff{4};

    % 전진비행 분석 결과가 있으면 참고 속도로 사용
    if exist('V_ff','var') && ~isempty(V_ff)
        V_ref_forward = V_ff;
    elseif exist('V_max_best','var') && ~isempty(V_max_best)
        V_ref_forward = 0.95 * V_max_best;
    else
        V_ref_forward = NaN;
    end

else
    analysis_mode   = 'Hover optimum propeller';
    prop_name_ge    = propSpecification{1};
    prop_diam_in_ge = propSpecification{2};
    prop_pitch_in_ge = propSpecification{3};

    V_ref_forward = NaN;
end

%% -------------------------------------------------------------------------
% [설계 기준]
% -------------------------------------------------------------------------
% fGE_upper_limit:
%   안정적 착륙을 위한 실용 상한 기준.
%
% gradient_stop:
%   높이를 더 키워도 fGE 감소량이 충분히 작아지면,
%   목표 fGE에 억지로 맞추지 않고 해당 지점에서 멈추기 위한 기준.

fGE_stable_limit = 1.05;
fGE_target_ref   = 1.08;
fGE_upper_limit  = 1.10;

gradient_stop_threshold = 0.9;   % [1/m], 약 0.009 fGE 변화 / 1 cm
min_stop_height_m       = 0.1;  % [m], 너무 낮은 높이에서 조기 stop 방지

%% -------------------------------------------------------------------------


if exist('best_arm_len_p','var') && ~isempty(best_arm_len_p) && ~isnan(best_arm_len_p)
    % planar_analysis.m의 best_arm_len_p는 mm 단위로 계산된다.
    % ground effect 계산은 SI 단위[m]를 사용하므로 mm -> m로 변환한다.
    ArmLength_m = best_arm_len_p / 1000;
    geometry_source = 'planar_analysis result: best_arm_len_p [mm] converted to m';

elseif exist('best_arm_len_n','var') && ~isempty(best_arm_len_n) && ~isnan(best_arm_len_n)
    % 비평면(non-planar) 분석값을 사용할 경우에도 mm -> m로 변환한다.
    ArmLength_m = best_arm_len_n / 1000;
    geometry_source = 'planar_analysis result: best_arm_len_n [mm] converted to m';

else
    ArmLength_m = 0.25;
    geometry_source = 'default S500 assumption: ArmLength = 0.25 m';
end

d_m = sqrt(2) * ArmLength_m;
b_m = 2 * ArmLength_m;

% Body/fountain effect 계수.
% 논문에서는 특정 PQUAD 실험기체에 대해 Kb ≈ 2를 사용했지만,
% S500에 그대로 적용하면 과대평가 가능성이 있어 0.5로 완화.
Kb = 0.5;

%% -------------------------------------------------------------------------
% [프롭 및 호버 유도속도 계산]
% -------------------------------------------------------------------------
rho = 1.225;   % [kg/m^3]
g   = 9.81;    % [m/s^2]

R_m_ge = prop_diam_in_ge * 0.0254 / 2;
A_m2   = pi * R_m_ge^2;

T_total_N     = mass_Total / 1000 * g;
T_per_rotor_N = T_total_N / RotorNo;

vi_hover = sqrt(T_per_rotor_N / (2 * rho * A_m2));

%% -------------------------------------------------------------------------
% [실용 랜딩기어 높이 범위]
% -------------------------------------------------------------------------
% z는 물리적인 랜딩기어 다리 길이 그 자체가 아니라,
% 바닥에서 로터 디스크면까지의 높이.
%
% S500 + 13 inch 이하급 프롭을 고려하여 10~25 cm 범위를 분석.

z_min_m = 0.10;
z_max_m = 0.25;

z_m_vec = linspace(z_min_m, z_max_m, 200)';

%% -------------------------------------------------------------------------
% [호버 기준 지면효과 계산]
% -------------------------------------------------------------------------
fGE_hover_vec = computeQuadGroundEffect(z_m_vec, R_m_ge, d_m, b_m, Kb, 0);

%% -------------------------------------------------------------------------
% [Gradient stop 계산]
% -------------------------------------------------------------------------
dfGE_dz = gradient(fGE_hover_vec, z_m_vec);
sensitivity_abs = abs(dfGE_dz);

idx_gradient_stop = find( ...
    z_m_vec >= min_stop_height_m & ...
    sensitivity_abs <= gradient_stop_threshold, ...
    1, 'first');

if ~isempty(idx_gradient_stop)
    gradient_stop_height_m = z_m_vec(idx_gradient_stop);
    fGE_gradient_stop = fGE_hover_vec(idx_gradient_stop);
    gradient_stop_sensitivity = sensitivity_abs(idx_gradient_stop);
else
    gradient_stop_height_m = NaN;
    fGE_gradient_stop = NaN;
    gradient_stop_sensitivity = NaN;
end

%% -------------------------------------------------------------------------
% [기준 fGE 도달 높이 계산]
% -------------------------------------------------------------------------
stableIdx = find(fGE_hover_vec <= fGE_stable_limit, 1, 'first');
if ~isempty(stableIdx)
    stable_height_m = z_m_vec(stableIdx);
else
    stable_height_m = NaN;
end

targetIdx = find(fGE_hover_vec <= fGE_target_ref, 1, 'first');
if ~isempty(targetIdx)
    target_ref_height_m = z_m_vec(targetIdx);
else
    target_ref_height_m = NaN;
end

upperIdx = find(fGE_hover_vec <= fGE_upper_limit, 1, 'first');
if ~isempty(upperIdx)
    upper_limit_height_m = z_m_vec(upperIdx);
else
    upper_limit_height_m = NaN;
end

%% -------------------------------------------------------------------------
% [최종 랜딩기어 높이 결정]
% -------------------------------------------------------------------------
% 우선순위:
%   1. gradient stop이 발견되면 그 지점을 최종 추천값으로 사용
%      → 더 높여도 지면효과 감소 효율이 작다고 판단
%
%   2. gradient stop이 없고 fGE <= upper limit에 도달하면 그 높이 사용
%
%   3. 둘 다 없으면 실용 범위의 최대 높이 사용

% 최종 랜딩기어 높이 결정 로직
%
% 기존처럼 우선순위를 순차적으로 적용하는 대신,
% 아래 후보 높이들 중 가장 작은 값을 최종 높이로 선택한다:
%
%   1) gradient stop 높이
%   2) fGE <= upper limit 를 만족하는 높이
%   3) 최대 허용 높이 z_max_m
%
% 즉, 허용 가능한 설계 조건들 중
% 가장 먼저 만족되는(가장 낮은) 높이를 최종 추천값으로 사용한다.
candidate_heights_m = [gradient_stop_height_m, upper_limit_height_m, z_max_m];
candidate_names = {'gradient stop', sprintf('fGE <= %.2f', fGE_upper_limit), 'max practical height'};

valid_candidate_idx = find(~isnan(candidate_heights_m));
[final_height_m, idx_local_min] = min(candidate_heights_m(valid_candidate_idx));
final_source_idx = valid_candidate_idx(idx_local_min);

final_fGE_hover = computeQuadGroundEffect(final_height_m, R_m_ge, d_m, b_m, Kb, 0);
final_reason = sprintf('minimum candidate height selected: %s', candidate_names{final_source_idx});

%% -------------------------------------------------------------------------
% [전진속도에 따른 지면효과 계산]
% -------------------------------------------------------------------------
% computeQuadGroundEffect의 마지막 입력은 V/vi.
% V=0이면 호버, V>0이면 전진속도 증가에 따른 지면효과 감소를 계산.
%
% 전진비행 최적화 결과 V_ff가 있으면 해당 속도에서의 fGE도 출력.

V_vec = linspace(0, V_ref_forward, 200)';
V_over_vi_vec = V_vec ./ vi_hover;

fGE_vsV = computeQuadGroundEffect(final_height_m, R_m_ge, d_m, b_m, Kb, V_over_vi_vec);

if ~isnan(V_ref_forward)
    fGE_forward_ref = computeQuadGroundEffect(final_height_m, R_m_ge, d_m, b_m, Kb, V_ref_forward / vi_hover);
else
    fGE_forward_ref = NaN;
end

%% -------------------------------------------------------------------------
% [결과 출력]
% -------------------------------------------------------------------------
fprintf('\n======= Ground Effect / Landing Gear Analysis =======\n');
fprintf('Analysis mode: %s\n', analysis_mode);
fprintf('Selected propeller: %s (%.1f in x %.1f in)\n', ...
    prop_name_ge, prop_diam_in_ge, prop_pitch_in_ge);

fprintf('Rotor radius: %.3f m (%.1f cm)\n', R_m_ge, R_m_ge * 100);

if ~exist('geometry_source','var')
    geometry_source = 'default S500 assumption: ArmLength = 0.25 m';
end

fprintf('S500 geometry: ArmLength = %.3f m, d = %.3f m, b = %.3f m\n', ...
    ArmLength_m, d_m, b_m);

fprintf('Tip-to-tip clearance: %.3f m (%.1f cm)\n', ...
    d_m - 2 * R_m_ge, (d_m - 2 * R_m_ge) * 100);

fprintf('Kb: %.2f\n', Kb);
fprintf('Hover induced velocity vi: %.2f m/s\n', vi_hover);

fprintf('Candidate rotor height range: %.1f ~ %.1f cm\n', ...
    z_min_m * 100, z_max_m * 100);

fprintf('fGE upper limit: %.2f\n', fGE_upper_limit);
fprintf('Gradient threshold: %.2f 1/m\n', gradient_stop_threshold);

if ~isnan(gradient_stop_height_m)
    fprintf('Gradient stop: %.1f cm (fGE_hover=%.3f, |dfGE/dz|=%.3f [1/m])\n', ...
        gradient_stop_height_m * 100, fGE_gradient_stop, gradient_stop_sensitivity);
else
    fprintf('Gradient stop: not found within candidate range.\n');
end

if ~isnan(upper_limit_height_m)
    fprintf('Reference fGE_hover <= %.2f height: %.1f cm\n', ...
        fGE_upper_limit, upper_limit_height_m * 100);
else
    fprintf('Reference fGE_hover <= %.2f height: not found within candidate range.\n', ...
        fGE_upper_limit);
end

if ~isnan(stable_height_m)
    fprintf('Reference fGE_hover <= %.2f height: %.1f cm\n', ...
        fGE_stable_limit, stable_height_m * 100);
else
    fprintf('Reference fGE_hover <= %.2f height: not found within candidate range.\n', ...
        fGE_stable_limit);
end

fprintf('Final recommended rotor height: %.1f cm (hover fGE=%.3f)\n', ...
    final_height_m * 100, final_fGE_hover);

fprintf('Reason: %s\n', final_reason);

if ~isnan(V_ref_forward)
    fprintf('Forward-flight reference speed: %.2f m/s\n', V_ref_forward);
    fprintf('fGE at final height and forward speed: %.3f\n', fGE_forward_ref);
else
    fprintf('Forward-flight reference speed: not available. Only speed sweep plot is generated.\n');
end

fprintf('=====================================================\n');

%% -------------------------------------------------------------------------
% [Plot 1] 호버 비행 시 높이에 따른 지면효과
% -------------------------------------------------------------------------
figure('Name','Ground Effect vs Rotor Height');
hCurve = plot(z_m_vec * 100, fGE_hover_vec, 'LineWidth', 1.5);
hold on;

hlUpper = yline(fGE_upper_limit, '--', sprintf('Upper limit %.2f', fGE_upper_limit), ...
    'HandleVisibility', 'off');

if ~isnan(gradient_stop_height_m)
    hStop = plot(gradient_stop_height_m * 100, fGE_gradient_stop, 'ro', ...
        'MarkerSize', 8, 'LineWidth', 2);
    
    gradient_stop_slope = dfGE_dz(idx_gradient_stop);
    
    deltaZ_tan = (z_m_vec(end) - z_m_vec(1)) / 20;
    z_tan = [gradient_stop_height_m - deltaZ_tan, gradient_stop_height_m + deltaZ_tan];
    fGE_tan = fGE_gradient_stop + gradient_stop_slope * (z_tan - gradient_stop_height_m);
    
    plot(z_tan * 100, fGE_tan, 'k:', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    
    text((gradient_stop_height_m * 100) - 0.5, fGE_gradient_stop - 0.008, ...
        sprintf('df/dz = %.3f [1/m]', gradient_stop_slope), ...
        'VerticalAlignment', 'top', 'HorizontalAlignment', 'right');
end

hFinal = plot(final_height_m * 100, final_fGE_hover, 'ks', ...
    'MarkerSize', 8, 'LineWidth', 2);

    text((final_height_m * 100) + 0.5, final_fGE_hover + 0.01, ...
        sprintf('f_{GE}=%.3f', final_fGE_hover), ...
        'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');

if ~isnan(upper_limit_height_m)
    if exist('upperIdx', 'var') && ~isempty(upperIdx) && ~isnan(upperIdx)
        fGE_upper_height_val = fGE_hover_vec(upperIdx);
    else
        fGE_upper_height_val = computeQuadGroundEffect(upper_limit_height_m, R_m_ge, d_m, b_m, Kb, 0);
    end
    hUpperMarker = plot(upper_limit_height_m * 100, fGE_upper_height_val, 'gd', ...
        'MarkerSize', 7, 'LineWidth', 1.5);
end
if ~isnan(stable_height_m)
    if exist('stableIdx', 'var') && ~isempty(stableIdx) && ~isnan(stableIdx)
        fGE_stable_height_val = fGE_hover_vec(stableIdx);
    else
        fGE_stable_height_val = computeQuadGroundEffect(stable_height_m, R_m_ge, d_m, b_m, Kb, 0);
    end
    hStableMarker = plot(stable_height_m * 100, fGE_stable_height_val, 'bd', ...
        'MarkerSize', 7, 'LineWidth', 1.5);
end

yLimits = ylim;
x_final_cm = final_height_m * 100;
plot([x_final_cm x_final_cm], [yLimits(1), final_fGE_hover], 'k--', 'HandleVisibility', 'off');

ax = gca;
currentTicks = get(ax, 'XTick');

if all(abs(currentTicks - x_final_cm) > 1e-6)
    newTicks = sort([currentTicks, x_final_cm]);
else
    newTicks = currentTicks;
end

set(ax, 'XTick', newTicks);

newTickLabels = cell(size(newTicks));
for kk = 1:numel(newTicks)
    if abs(newTicks(kk) - x_final_cm) < 1e-6
        newTickLabels{kk} = sprintf('%.1f', newTicks(kk));
    elseif abs(newTicks(kk) - round(newTicks(kk))) < 1e-6
        newTickLabels{kk} = sprintf('%.0f', newTicks(kk));
    else
        newTickLabels{kk} = sprintf('%.1f', newTicks(kk));
    end
end
set(ax, 'XTickLabel', newTickLabels);

xlabel('Rotor height z [cm]');
ylabel('Ground effect factor f_{GE}');
title(sprintf('Ground Effect vs Height - %s', prop_name_ge));
grid on;
hold off;

legendHandles = [];
legendStrings = {};

if exist('hCurve', 'var') && isgraphics(hCurve)
    legendHandles = [legendHandles, hCurve];
    legendStrings{end+1} = 'fGE curve';
end

if exist('hUpperMarker', 'var') && isgraphics(hUpperMarker)
    legendHandles = [legendHandles, hUpperMarker];
    legendStrings{end+1} = 'Upper limit crossing';
end

if exist('hStableMarker', 'var') && isgraphics(hStableMarker)
    legendHandles = [legendHandles, hStableMarker];
    legendStrings{end+1} = 'Stable limit crossing';
end

if exist('hStop', 'var') && isgraphics(hStop)
    legendHandles = [legendHandles, hStop];
    legendStrings{end+1} = 'Gradient stop';
end

if exist('hFinal', 'var') && isgraphics(hFinal)
    legendHandles = [legendHandles, hFinal];
    legendStrings{end+1} = 'Final recommended';
end

legend(legendHandles, legendStrings, 'Location', 'northeast');

%% -------------------------------------------------------------------------
% [Plot 2] 전진속도에 따른 지면효과 (고정된 운용 높이)
% -------------------------------------------------------------------------
% 전진비행 최적 프롭을 분석하는 경우에만 전진속도 스윕 그래프를 생성한다.
% 호버 최적 프롭의 지면효과 분석에서는 이 그래프를 출력하지 않는다.
if strcmp(analysis_mode, 'Forward-flight optimum propeller') 
    % 2 m 높이에서 전진속도에 따른 지면효과를 평가한다.  
    % 시뮬레이션 높이 analysis_height_m 값을 변경하면 다른 운용 높이에 대해 분석할 수 있다.
    analysis_height_m = 2.0;  % [m] 

    if isnan(V_ref_forward)
        V_plot_max = 12;
    else
        V_plot_max = V_ref_forward;
    end
    V_vec_forw = linspace(0, V_plot_max, 200)';
    V_over_vi_vec_forw = V_vec_forw ./ vi_hover;
    fGE_vsV_analysis = computeQuadGroundEffect(analysis_height_m, R_m_ge, d_m, b_m, Kb, V_over_vi_vec_forw);

    if ~isnan(V_ref_forward)
        fGE_forward_ref_analysis = computeQuadGroundEffect(analysis_height_m, R_m_ge, d_m, b_m, Kb, V_ref_forward / vi_hover);
    else
        fGE_forward_ref_analysis = NaN;
    end

    figure('Name','Ground Effect vs Forward Speed');
    plot(V_vec_forw, fGE_vsV_analysis, 'LineWidth', 1.5);
    hold on;

    yline(1.0, '--', 'No ground effect', 'HandleVisibility', 'off');
    yline(fGE_vsV_analysis(1), '-.', sprintf('Hover fGE %.3f', fGE_vsV_analysis(1)), 'HandleVisibility', 'off');

    if ~isnan(V_ref_forward)
        plot(V_ref_forward, fGE_forward_ref_analysis, 'ro', ...
            'MarkerSize', 8, 'LineWidth', 2);
    end

    xlabel('Forward speed V [m/s]');
    ylabel('Ground effect factor f_{GE}');
    title(sprintf('Ground Effect vs Forward Speed at z = %.1f m', analysis_height_m));
    grid on;
    hold off;
end

%%
function fGE = computeQuadGroundEffect(z, R, d, b, Kb, V_over_vi)
    z         = z(:);
    V_over_vi = V_over_vi(:);

    term_single   = (R ./ (4 .* z)).^2;
    term_adjacent = R^2 .* z ./ (sqrt(d^2   + 4.*z.^2).^3);
    term_diagonal = (R^2/2) .* z ./ (sqrt(2.*d^2 + 4.*z.^2).^3);
    term_body     = 2.*R^2 .* z ./ (sqrt(b^2   + 4.*z.^2).^3) .* Kb;
    GE_terms      = term_single + term_adjacent + term_diagonal + term_body;

    if numel(z) == 1 && numel(V_over_vi) > 1
        GE_rep = repmat(GE_terms, size(V_over_vi));
        chi    = 1 ./ (1 + V_over_vi.^2);
        denom  = 1 - chi .* GE_rep;

    elseif numel(z) > 1 && numel(V_over_vi) == 1
        chi   = 1 ./ (1 + V_over_vi.^2);
        denom = 1 - chi .* GE_terms;

    elseif numel(z) == numel(V_over_vi)
        chi   = 1 ./ (1 + V_over_vi.^2);
        denom = 1 - chi .* GE_terms;

    else
        error('computeQuadGroundEffect:SizeMismatch', ...
            'z and V_over_vi must be scalar, one vector, or equal-length vectors.');
    end

    fGE = NaN(size(denom));
    valid = denom > 0;
    fGE(valid) = 1 ./ denom(valid);
end