%% =========================================================================
% ground_effect_analysis.m
%
% 실행 순서:
%
%   [호버 최적 프롭 기준 지면효과 분석]
%       main → arm_wake_analysis → planar_analysis → ground_effect_analysis
%
%   [전진비행 최적 프롭 기준 지면효과 분석]
%       main → forward_flight_analysis → arm_wake_analysis
%            → planar_analysis → ground_effect_analysis
%
% 목적:
%   planar_analysis.m 이 결정한 암 길이(best_arm_len_p)와 최종 AUM(final_aum_p)
%   기반으로 지면효과 계수를 계산하고 최적 랜딩기어(로터 높이)를 결정한다.
%
% [planar_analysis에서 수신하는 변수]
%   best_arm_len_p  — 선정된 암 길이 (중심→프롭, mm)
%   final_aum_p     — 암 무게 포함 최종 AUM (g)
%
% =========================================================================

%% -------------------------------------------------------------------------
% [사전 확인]
% -------------------------------------------------------------------------
if ~exist('propSpecification','var') || ~exist('mass_Total','var') || ~exist('RotorNo','var')
    error('main.m을 먼저 실행해야 합니다. 필요한 변수: propSpecification, mass_Total, RotorNo');
end
if ~exist('best_arm_len_p','var') || ~exist('final_aum_p','var')
    error(['planar_analysis.m을 먼저 실행해야 합니다.\n' ...
           '필요한 변수: best_arm_len_p, final_aum_p']);
end

%% -------------------------------------------------------------------------
% [분석 대상 프롭 자동 선택]
% -------------------------------------------------------------------------
if exist('prop_ff','var') && ~isempty(prop_ff)
    analysis_mode    = 'Forward-flight optimum propeller';
    prop_name_ge     = prop_ff{1};
    prop_diam_in_ge  = prop_ff{3};
    prop_pitch_in_ge = prop_ff{4};

    if exist('V_ff','var') && ~isempty(V_ff)
        V_ref_forward = V_ff;
    elseif exist('V_max_best','var') && ~isempty(V_max_best)
        V_ref_forward = 0.95 * V_max_best;
    else
        V_ref_forward = NaN;
    end
else
    analysis_mode    = 'Hover optimum propeller';
    prop_name_ge     = propSpecification{1};
    prop_diam_in_ge  = propSpecification{2};
    prop_pitch_in_ge = propSpecification{3};
    V_ref_forward    = NaN;
end

%% -------------------------------------------------------------------------
% [설계 기준]
% -------------------------------------------------------------------------
fGE_stable_limit        = 1.05;
fGE_target_ref          = 1.08;
fGE_upper_limit         = 1.10;
gradient_stop_threshold = 0.9;   % [1/m]
min_stop_height_m       = 0.1;   % [m]

%% -------------------------------------------------------------------------
% [기체 형상 — planar_analysis 결과 사용]
% -------------------------------------------------------------------------
% best_arm_len_p : planar_analysis에서 결정된 암 길이 (중심→프롭) [mm]
% final_aum_p    : 암 무게 포함 최종 AUM [g]
%
% 지면효과 계산에 필요한 기하 파라미터:
%   ArmLength_m  : 암 길이 (중심→프롭) [m]
%   d_m          : 인접 로터 간 거리 = √2 × ArmLength_m  [m]
%   b_m          : 대각 로터 간 거리 = 2  × ArmLength_m  [m]
%   (정사각형 X자 배치 가정)

if ~isnan(best_arm_len_p) && best_arm_len_p > 0
    ArmLength_m     = best_arm_len_p / 1000;   % mm → m
    geometry_source = sprintf( ...
        'planar_analysis: best_arm_len_p = %.2f mm → %.4f m', ...
        best_arm_len_p, ArmLength_m);
else
    ArmLength_m     = 0.25;
    geometry_source = 'fallback default: ArmLength = 0.25 m (best_arm_len_p 미설정)';
    warning('ground_effect_analysis: best_arm_len_p 가 NaN/0. 기본값 0.25 m 사용.');
end

d_m = sqrt(2) * ArmLength_m;   % 인접 로터 간 거리
b_m = 2       * ArmLength_m;   % 대각 로터 간 거리

% Body/fountain effect 계수 (S500급 완화 적용)
Kb = 0.5;

%% -------------------------------------------------------------------------
% [프롭 및 호버 유도속도 계산]
% -------------------------------------------------------------------------
rho = 1.225;   % [kg/m^3]
g   = 9.81;    % [m/s^2]

R_m_ge = prop_diam_in_ge * 0.0254 / 2;   % 로터 반경 [m]
A_m2   = pi * R_m_ge^2;                   % 로터 디스크 면적 [m²]

% 추력 계산 기준: final_aum_p (암 무게 포함 최종 AUM)
T_total_N     = final_aum_p / 1000 * g;   % 총 추력 [N]
T_per_rotor_N = T_total_N / RotorNo;       % 로터 1개당 추력 [N]

vi_hover = sqrt(T_per_rotor_N / (2 * rho * A_m2));   % 유도속도 [m/s]

%% -------------------------------------------------------------------------
% [실용 랜딩기어 높이 범위]
% -------------------------------------------------------------------------
% z: 지면에서 로터 디스크면까지의 높이
% S500 + 13 inch 이하급 프롭 기준 10~25 cm

z_min_m  = 0.10;
z_max_m  = 0.25;
z_m_vec  = linspace(z_min_m, z_max_m, 200)';

%% -------------------------------------------------------------------------
% [호버 기준 지면효과 계산]
% -------------------------------------------------------------------------
fGE_hover_vec = computeQuadGroundEffect(z_m_vec, R_m_ge, d_m, b_m, Kb, 0);

%% -------------------------------------------------------------------------
% [Gradient stop 계산]
% -------------------------------------------------------------------------
dfGE_dz         = gradient(fGE_hover_vec, z_m_vec);
sensitivity_abs = abs(dfGE_dz);

idx_gradient_stop = find( ...
    z_m_vec >= min_stop_height_m & ...
    sensitivity_abs <= gradient_stop_threshold, ...
    1, 'first');

if ~isempty(idx_gradient_stop)
    gradient_stop_height_m    = z_m_vec(idx_gradient_stop);
    fGE_gradient_stop         = fGE_hover_vec(idx_gradient_stop);
    gradient_stop_sensitivity = sensitivity_abs(idx_gradient_stop);
else
    gradient_stop_height_m    = NaN;
    fGE_gradient_stop         = NaN;
    gradient_stop_sensitivity = NaN;
end

%% -------------------------------------------------------------------------
% [기준 fGE 도달 높이 계산]
% -------------------------------------------------------------------------
stableIdx = find(fGE_hover_vec <= fGE_stable_limit, 1, 'first');
stable_height_m = NaN;
if ~isempty(stableIdx), stable_height_m = z_m_vec(stableIdx); end

targetIdx = find(fGE_hover_vec <= fGE_target_ref, 1, 'first');
target_ref_height_m = NaN;
if ~isempty(targetIdx), target_ref_height_m = z_m_vec(targetIdx); end

upperIdx = find(fGE_hover_vec <= fGE_upper_limit, 1, 'first');
upper_limit_height_m = NaN;
if ~isempty(upperIdx), upper_limit_height_m = z_m_vec(upperIdx); end

%% -------------------------------------------------------------------------
% [최종 랜딩기어 높이 결정]
% -------------------------------------------------------------------------
% 후보 높이 중 가장 작은(낮은) 값을 최종 추천값으로 선택한다.
%   1) gradient stop 높이
%   2) fGE <= upper limit 를 만족하는 높이
%   3) 최대 허용 높이 z_max_m
candidate_heights_m = [gradient_stop_height_m, upper_limit_height_m, z_max_m];
candidate_names     = {'gradient stop', ...
                       sprintf('fGE <= %.2f', fGE_upper_limit), ...
                       'max practical height'};

valid_idx = find(~isnan(candidate_heights_m));
[final_height_m, idx_local_min] = min(candidate_heights_m(valid_idx));
final_source_idx = valid_idx(idx_local_min);

final_fGE_hover = computeQuadGroundEffect(final_height_m, R_m_ge, d_m, b_m, Kb, 0);
final_reason    = sprintf('minimum candidate height selected: %s', ...
                           candidate_names{final_source_idx});

%% -------------------------------------------------------------------------
% [전진속도에 따른 지면효과 계산]
% -------------------------------------------------------------------------
if isnan(V_ref_forward)
    V_vec_ge = linspace(0, 12, 200)';
else
    V_vec_ge = linspace(0, V_ref_forward, 200)';
end
V_over_vi_vec = V_vec_ge ./ vi_hover;

fGE_vsV = computeQuadGroundEffect(final_height_m, R_m_ge, d_m, b_m, Kb, V_over_vi_vec);

if ~isnan(V_ref_forward)
    fGE_forward_ref = computeQuadGroundEffect( ...
        final_height_m, R_m_ge, d_m, b_m, Kb, V_ref_forward / vi_hover);
else
    fGE_forward_ref = NaN;
end

%% -------------------------------------------------------------------------
% [결과 출력]
% -------------------------------------------------------------------------
fprintf('\n======================================================\n');
fprintf('[Ground Effect / Landing Gear Analysis]\n');
fprintf('======================================================\n');
fprintf('Analysis mode         : %s\n', analysis_mode);
fprintf('Selected propeller    : %s (%.1f in × %.1f in)\n', ...
    prop_name_ge, prop_diam_in_ge, prop_pitch_in_ge);
fprintf('Rotor radius          : %.4f m (%.1f cm)\n', R_m_ge, R_m_ge * 100);
fprintf('------------------------------------------------------\n');
fprintf('[기하 — planar_analysis 연동]\n');
fprintf('  %s\n', geometry_source);
fprintf('  ArmLength  : %.4f m (%.2f mm)\n', ArmLength_m, ArmLength_m * 1000);
fprintf('  d (인접)   : %.4f m (%.2f mm)\n', d_m, d_m * 1000);
fprintf('  b (대각)   : %.4f m (%.2f mm)\n', b_m, b_m * 1000);
fprintf('  Tip clearance: %.4f m (%.1f cm)\n', ...
    d_m - 2 * R_m_ge, (d_m - 2 * R_m_ge) * 100);
fprintf('------------------------------------------------------\n');
fprintf('[추력 / 유도속도 — final_aum_p 기준]\n');
fprintf('  final_aum_p (암 포함 최종 AUM) : %.1f g\n', final_aum_p);
fprintf('  T_total                         : %.3f N\n', T_total_N);
fprintf('  T_per_rotor                     : %.4f N\n', T_per_rotor_N);
fprintf('  vi_hover                        : %.3f m/s\n', vi_hover);
fprintf('------------------------------------------------------\n');
fprintf('Kb                    : %.2f\n', Kb);
fprintf('Candidate range       : %.1f ~ %.1f cm\n', z_min_m*100, z_max_m*100);
fprintf('fGE upper limit       : %.2f\n', fGE_upper_limit);
fprintf('Gradient threshold    : %.2f 1/m\n', gradient_stop_threshold);
fprintf('------------------------------------------------------\n');

if ~isnan(gradient_stop_height_m)
    fprintf('Gradient stop         : %.1f cm  (fGE=%.4f, |dfGE/dz|=%.4f 1/m)\n', ...
        gradient_stop_height_m*100, fGE_gradient_stop, gradient_stop_sensitivity);
else
    fprintf('Gradient stop         : not found within candidate range.\n');
end

if ~isnan(upper_limit_height_m)
    fprintf('fGE <= %.2f height    : %.1f cm\n', fGE_upper_limit, upper_limit_height_m*100);
else
    fprintf('fGE <= %.2f height    : not found within candidate range.\n', fGE_upper_limit);
end

if ~isnan(stable_height_m)
    fprintf('fGE <= %.2f height    : %.1f cm\n', fGE_stable_limit, stable_height_m*100);
else
    fprintf('fGE <= %.2f height    : not found within candidate range.\n', fGE_stable_limit);
end

fprintf('------------------------------------------------------\n');
fprintf('Final rotor height    : %.1f cm  (hover fGE=%.4f)\n', ...
    final_height_m*100, final_fGE_hover);
fprintf('Reason                : %s\n', final_reason);

if ~isnan(V_ref_forward)
    fprintf('Forward speed ref     : %.2f m/s\n', V_ref_forward);
    fprintf('fGE @ fwd speed       : %.4f\n', fGE_forward_ref);
else
    fprintf('Forward speed ref     : not available.\n');
end
fprintf('======================================================\n\n');

%% -------------------------------------------------------------------------
% [Plot 1] 호버 비행 시 높이에 따른 지면효과
% -------------------------------------------------------------------------
figure('Name','Ground Effect vs Rotor Height', 'Color','white', ...
       'Position',[100 100 860 480]);
hCurve = plot(z_m_vec * 100, fGE_hover_vec, 'b-', 'LineWidth', 1.8);
hold on;

yline(fGE_upper_limit, 'r--', sprintf('Upper limit %.2f', fGE_upper_limit), ...
    'HandleVisibility','off', 'LineWidth', 1.2);

legendHandles = hCurve;
legendStrings = {'fGE curve (hover)'};

if ~isnan(gradient_stop_height_m)
    hStop = plot(gradient_stop_height_m*100, fGE_gradient_stop, 'ro', ...
        'MarkerSize', 8, 'LineWidth', 2);
    gradient_stop_slope = dfGE_dz(idx_gradient_stop);
    dz_tan = (z_m_vec(end) - z_m_vec(1)) / 20;
    z_tan  = [gradient_stop_height_m - dz_tan, gradient_stop_height_m + dz_tan];
    fGE_tan = fGE_gradient_stop + gradient_stop_slope*(z_tan - gradient_stop_height_m);
    plot(z_tan*100, fGE_tan, 'k:', 'LineWidth', 1.2, 'HandleVisibility','off');
    text((gradient_stop_height_m*100) - 0.5, fGE_gradient_stop - 0.008, ...
        sprintf('df/dz = %.3f [1/m]', gradient_stop_slope), ...
        'VerticalAlignment','top','HorizontalAlignment','right','FontSize',9);
    legendHandles(end+1) = hStop;
    legendStrings{end+1} = 'Gradient stop';
end

if ~isnan(upper_limit_height_m)
    fGE_ul_val = fGE_hover_vec(upperIdx);
    hUpperMarker = plot(upper_limit_height_m*100, fGE_ul_val, 'gd', ...
        'MarkerSize', 7, 'LineWidth', 1.5);
    legendHandles(end+1) = hUpperMarker;
    legendStrings{end+1} = sprintf('fGE=%.2f crossing', fGE_upper_limit);
end

if ~isnan(stable_height_m)
    fGE_st_val = fGE_hover_vec(stableIdx);
    hStableMarker = plot(stable_height_m*100, fGE_st_val, 'bd', ...
        'MarkerSize', 7, 'LineWidth', 1.5);
    legendHandles(end+1) = hStableMarker;
    legendStrings{end+1} = sprintf('fGE=%.2f crossing', fGE_stable_limit);
end

hFinal = plot(final_height_m*100, final_fGE_hover, 'ks', ...
    'MarkerSize', 9, 'LineWidth', 2);
text((final_height_m*100) + 0.4, final_fGE_hover + 0.008, ...
    sprintf('z=%.1fcm\nf_{GE}=%.3f', final_height_m*100, final_fGE_hover), ...
    'HorizontalAlignment','left','VerticalAlignment','bottom','FontSize',9);
legendHandles(end+1) = hFinal;
legendStrings{end+1} = 'Final recommended';

yLimits = ylim;
x_final_cm = final_height_m * 100;
plot([x_final_cm x_final_cm], [yLimits(1), final_fGE_hover], ...
    'k--', 'LineWidth', 1.0, 'HandleVisibility','off');

% X축 틱에 최종 높이 추가
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
        newTickLabels{kk} = sprintf('%.1f★', newTicks(kk));
    else
        newTickLabels{kk} = sprintf('%.0f', newTicks(kk));
    end
end
set(ax, 'XTickLabel', newTickLabels);

xlabel('Rotor height z [cm]', 'FontSize', 12);
ylabel('Ground effect factor f_{GE}', 'FontSize', 12);
title(sprintf('[Ground Effect vs Height]\n%s | ArmLen=%.1fmm, AUM=%.0fg', ...
    prop_name_ge, best_arm_len_p, final_aum_p), 'FontSize', 11);
legend(legendHandles, legendStrings, 'Location','northeast', 'FontSize', 10, 'Box','off');
grid on; hold off;

%% -------------------------------------------------------------------------
% [Plot 2] 전진속도에 따른 지면효과 (전진비행 최적 프롭 분석 시만 출력)
% -------------------------------------------------------------------------
if strcmp(analysis_mode, 'Forward-flight optimum propeller')
    analysis_height_m = 2.0;  % [m]

    if isnan(V_ref_forward)
        V_plot_max = 12;
    else
        V_plot_max = V_ref_forward;
    end
    V_vec_forw        = linspace(0, V_plot_max, 200)';
    V_ovi_forw        = V_vec_forw ./ vi_hover;
    fGE_vsV_analysis  = computeQuadGroundEffect( ...
        analysis_height_m, R_m_ge, d_m, b_m, Kb, V_ovi_forw);

    if ~isnan(V_ref_forward)
        fGE_fwd_ref_analysis = computeQuadGroundEffect( ...
            analysis_height_m, R_m_ge, d_m, b_m, Kb, V_ref_forward / vi_hover);
    else
        fGE_fwd_ref_analysis = NaN;
    end

    figure('Name','Ground Effect vs Forward Speed', 'Color','white', ...
           'Position',[200 200 760 420]);
    plot(V_vec_forw, fGE_vsV_analysis, 'b-', 'LineWidth', 1.8);
    hold on;
    yline(1.0, 'k--', 'No ground effect', 'HandleVisibility','off', 'LineWidth', 1.0);
    yline(fGE_vsV_analysis(1), 'r-.', ...
        sprintf('Hover fGE %.3f', fGE_vsV_analysis(1)), ...
        'HandleVisibility','off', 'LineWidth', 1.0);

    if ~isnan(V_ref_forward)
        plot(V_ref_forward, fGE_fwd_ref_analysis, 'ro', ...
            'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', ...
            sprintf('V_{ref}=%.1fm/s, fGE=%.3f', V_ref_forward, fGE_fwd_ref_analysis));
        legend('Location','northeast', 'FontSize', 10, 'Box','off');
    end
    xlabel('Forward speed V [m/s]', 'FontSize', 12);
    ylabel('Ground effect factor f_{GE}', 'FontSize', 12);
    title(sprintf('Ground Effect vs Forward Speed  (z = %.1f m)', analysis_height_m), ...
        'FontSize', 11);
    grid on; hold off;
end

%% =========================================================================
% 로컬 함수
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

    fGE   = NaN(size(denom));
    valid = denom > 0;
    fGE(valid) = 1 ./ denom(valid);
end