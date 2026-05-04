%% =========================================================================
% ground_effect_analysis.m
%
% 실행 순서:
%
%   [호버 최적 프롭 기준]
%       main → ground_effect_analysis
%
%   [전진비행 최적 프롭 기준]
%       main → forward_flight_analysis → ground_effect_analysis
%
% 목적:
%   선정된 프롭 기준으로 멀티로터 지면효과를 분석하고 랜딩기어 높이를 결정.
%   prop_ff(2안 프롭)가 workspace에 있으면 그것을 우선 사용하고,
%   없으면 propSpecification(1안 프롭)을 사용.
%
% 모델:
%   computeQuadGroundEffect (이 파일 하단 로컬 함수)
%   단일 로터 + 인접/대각 로터 간섭 + fountain 효과 포함.
%   aero_analysis의 단순 Cheeseman-Bennett 모델보다 멀티로터에 적합.
%
% 참조하는 Workspace 변수 (main.m 출력 필수):
%   propSpecification  — 1안 프롭 사양 {name, diam_in, pitch_in}
%   mass_Total         — 실측 이륙 중량 [g] (유도속도 계산 기준)
%   RotorNo            — 로터 수
%
% 참조하는 Workspace 변수 (forward_flight_analysis.m 출력, 있으면 사용):
%   prop_ff            — 2안 프롭 사양 (1×6 cell)
%   V_ff               — 2안 운용 전진속도 [m/s]
%   V_max_best         — 2안 최대 전진속도 [m/s]
% =========================================================================

%% 사전 확인: 필수 변수 존재 여부
required_vars = {'propSpecification', 'mass_Total', 'RotorNo'};
missing = {};
for v = required_vars
    if ~exist(v{1}, 'var'), missing{end+1} = v{1}; end
end
if ~isempty(missing)
    error('ground_effect_analysis: 다음 변수가 없습니다. main.m을 먼저 실행하세요.\n  누락: %s', ...
        strjoin(missing, ', '));
end

% computeQuadGroundEffect는 이 파일 하단에 로컬 함수로 정의되어 있음.
% exist(...,'file') 방식으로는 로컬 함수를 인식하지 못하므로
% 실제 호출 가능 여부로만 확인함 (실행 중 오류 발생 시 MATLAB이 알림).

%% 분석 대상 프롭 자동 선택
% forward_flight_analysis 실행 후 prop_ff가 있으면 2안 프롭 사용,
% 없으면 1안 프롭(propSpecification) 사용.
if exist('prop_ff','var') && ~isempty(prop_ff)
    analysis_mode    = 'Forward-flight optimum propeller (2안)';
    prop_name_ge     = prop_ff{1};
    prop_diam_in_ge  = prop_ff{3};
    prop_pitch_in_ge = prop_ff{4};

    if exist('V_ff','var') && ~isempty(V_ff) && ~isnan(V_ff)
        V_ref_forward = V_ff;
    elseif exist('V_max_best','var') && ~isempty(V_max_best) && ~isnan(V_max_best)
        V_ref_forward = 0.95 * V_max_best;
    else
        V_ref_forward = NaN;
    end
else
    analysis_mode    = 'Hover optimum propeller (1안)';
    prop_name_ge     = propSpecification{1};
    prop_diam_in_ge  = propSpecification{2};
    prop_pitch_in_ge = propSpecification{3};
    V_ref_forward    = NaN;
end

%% 설계 기준
% fGE_upper_limit: 안정적 착륙을 위한 실용 상한.
% gradient_stop_threshold: 높이를 더 올려도 fGE 감소량이 이 값보다 작아지면
%   목표 fGE에 억지로 맞추지 않고 해당 지점에서 결정.
fGE_stable_limit        = 1.05;
fGE_target_ref          = 1.08;
fGE_upper_limit         = 1.10;
gradient_stop_threshold = 0.8;    % [1/m], 약 0.008 fGE 변화 / 1 cm
min_stop_height_m       = 0.16;   % [m], 너무 낮은 높이에서 조기 stop 방지

%% S500 기하 파라미터
% ArmLength_m: 기체 중심 ~ 모터 축 거리 [m]
% X-quad 기준:
%   d_m: 인접 로터 축 사이 거리
%   b_m: 대각 로터 축 사이 거리
ArmLength_m = 0.25;
d_m = sqrt(2) * ArmLength_m;
b_m = 2 * ArmLength_m;

% Kb: fountain effect 보정 계수.
% 논문(PQUAD 실험기체)의 Kb=2를 S500에 그대로 적용하면 과대평가 우려 → 0.5로 완화.
Kb = 0.5;

%% 프롭 및 호버 유도속도 계산
rho = 1.225;   % [kg/m^3]
g   = 9.81;    % [m/s^2]

R_m_ge = prop_diam_in_ge * 0.0254 / 2;   % 프롭 반경 [m]
A_m2   = pi * R_m_ge^2;

% 유도속도 계산 기준: mass_Total (실측 보정값)
% thrustHover_Est(추정값)가 아닌 실측 중량을 써야 실제 착륙 조건에 맞음.
T_total_N     = mass_Total / 1000 * g;
T_per_rotor_N = T_total_N / RotorNo;
vi_hover      = sqrt(T_per_rotor_N / (2 * rho * A_m2));

%% 분석 높이 범위
% z: 바닥에서 로터 디스크면까지의 높이 (랜딩기어 다리 길이와 다름).
% S500 + 13 inch 이하급 프롭 기준 실용 범위 10~25 cm.
z_min_m = 0.10;
z_max_m = 0.25;
z_m_vec = linspace(z_min_m, z_max_m, 200)';

%% 호버 기준 지면효과 계산
fGE_hover_vec = computeQuadGroundEffect(z_m_vec, R_m_ge, d_m, b_m, Kb, 0);

%% Gradient stop 계산
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

%% 기준 fGE 도달 높이 계산
stableIdx = find(fGE_hover_vec <= fGE_stable_limit, 1, 'first');
stable_height_m = NaN;
if ~isempty(stableIdx), stable_height_m = z_m_vec(stableIdx); end

targetIdx = find(fGE_hover_vec <= fGE_target_ref, 1, 'first');
target_ref_height_m = NaN;
if ~isempty(targetIdx), target_ref_height_m = z_m_vec(targetIdx); end

upperIdx = find(fGE_hover_vec <= fGE_upper_limit, 1, 'first');
upper_limit_height_m = NaN;
if ~isempty(upperIdx), upper_limit_height_m = z_m_vec(upperIdx); end

%% 최종 랜딩기어 높이 결정
% 우선순위:
%   1. gradient stop 발견 → 더 높여도 효과 감소 효율이 낮다고 판단, 해당 높이 채택
%   2. fGE <= upper_limit 도달 → 해당 높이 채택
%   3. 둘 다 없으면 실용 범위 최대 높이 사용
if ~isnan(gradient_stop_height_m)
    final_height_m  = gradient_stop_height_m;
    final_fGE_hover = fGE_gradient_stop;
    final_reason    = sprintf('gradient stop: |dfGE/dz| <= %.2f 1/m', gradient_stop_threshold);

elseif ~isnan(upper_limit_height_m)
    final_height_m  = upper_limit_height_m;
    final_fGE_hover = computeQuadGroundEffect(final_height_m, R_m_ge, d_m, b_m, Kb, 0);
    final_reason    = sprintf('fGE <= %.2f reached', fGE_upper_limit);

else
    final_height_m  = z_max_m;
    final_fGE_hover = fGE_hover_vec(end);
    final_reason    = 'max practical height reached';
end

%% 전진속도에 따른 지면효과 계산
% V_ref_forward가 NaN이면 속도 스윕 없이 호버만 계산.
if ~isnan(V_ref_forward)
    V_vec         = linspace(0, V_ref_forward, 200)';
    V_over_vi_vec = V_vec ./ vi_hover;
    fGE_vsV       = computeQuadGroundEffect(final_height_m, R_m_ge, d_m, b_m, Kb, V_over_vi_vec);
    fGE_forward_ref = computeQuadGroundEffect( ...
        final_height_m, R_m_ge, d_m, b_m, Kb, V_ref_forward / vi_hover);
else
    % 전진속도 정보 없음: 호버 단일 포인트만 사용
    V_vec           = 0;
    fGE_vsV         = final_fGE_hover;
    fGE_forward_ref = NaN;
end

%% 결과 출력
fprintf('\n======= Ground Effect / Landing Gear Analysis =======\n');
fprintf('Analysis mode      : %s\n', analysis_mode);
fprintf('Selected propeller : %s (%.1f in x %.1f in)\n', ...
    prop_name_ge, prop_diam_in_ge, prop_pitch_in_ge);
fprintf('Rotor radius       : %.3f m (%.1f cm)\n', R_m_ge, R_m_ge * 100);
fprintf('S500 geometry      : ArmLength=%.3f m, d=%.3f m, b=%.3f m\n', ...
    ArmLength_m, d_m, b_m);
fprintf('Tip-to-tip clearance : %.3f m (%.1f cm)\n', ...
    d_m - 2*R_m_ge, (d_m - 2*R_m_ge)*100);
fprintf('Kb                 : %.2f\n', Kb);
fprintf('vi_hover           : %.2f m/s  (mass_Total=%.1fg 기준)\n', vi_hover, mass_Total);
fprintf('Height scan range  : %.1f ~ %.1f cm\n', z_min_m*100, z_max_m*100);
fprintf('fGE upper limit    : %.2f\n', fGE_upper_limit);
fprintf('Gradient threshold : %.2f 1/m\n', gradient_stop_threshold);

if ~isnan(gradient_stop_height_m)
    fprintf('Gradient stop      : %.1f cm  (fGE=%.3f, |dfGE/dz|=%.3f 1/m)\n', ...
        gradient_stop_height_m*100, fGE_gradient_stop, gradient_stop_sensitivity);
else
    fprintf('Gradient stop      : not found within range\n');
end

if ~isnan(upper_limit_height_m)
    fprintf('fGE <= %.2f height : %.1f cm\n', fGE_upper_limit, upper_limit_height_m*100);
else
    fprintf('fGE <= %.2f height : not found within range\n', fGE_upper_limit);
end

if ~isnan(target_ref_height_m)
    fprintf('fGE <= %.2f height : %.1f cm\n', fGE_target_ref, target_ref_height_m*100);
else
    fprintf('fGE <= %.2f height : not found within range\n', fGE_target_ref);
end

if ~isnan(stable_height_m)
    fprintf('fGE <= %.2f height : %.1f cm\n', fGE_stable_limit, stable_height_m*100);
else
    fprintf('fGE <= %.2f height : not found within range\n', fGE_stable_limit);
end

fprintf('Final height       : %.1f cm  (hover fGE=%.3f)\n', ...
    final_height_m*100, final_fGE_hover);
fprintf('Reason             : %s\n', final_reason);

if ~isnan(V_ref_forward)
    fprintf('Forward ref speed  : %.2f m/s  →  fGE=%.3f\n', V_ref_forward, fGE_forward_ref);
else
    fprintf('Forward ref speed  : N/A (forward_flight_analysis 미실행)\n');
end
fprintf('=====================================================\n');

%% Plot 1: 높이에 따른 지면효과
figure('Name','Ground Effect vs Rotor Height');
plot(z_m_vec*100, fGE_hover_vec, 'b-', 'LineWidth', 1.5);
hold on;
yline(fGE_upper_limit, 'r--', sprintf('Upper limit %.2f', fGE_upper_limit));
yline(fGE_target_ref,  'k-.', sprintf('Reference %.2f',   fGE_target_ref));
yline(fGE_stable_limit,'g--', sprintf('Stable %.2f',      fGE_stable_limit));
if ~isnan(gradient_stop_height_m)
    plot(gradient_stop_height_m*100, fGE_gradient_stop, 'ro', ...
        'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Gradient stop');
end
plot(final_height_m*100, final_fGE_hover, 'ks', ...
    'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Final recommendation');
hold off; grid on;
xlabel('Rotor height z [cm]');
ylabel('Ground effect factor f_{GE}');
title(sprintf('Ground Effect vs Height — %s', prop_name_ge));
legend('Location','northeast');

%% Plot 2: 전진속도에 따른 지면효과 (최종 높이 고정)
figure('Name','Ground Effect vs Forward Speed');
if ~isnan(V_ref_forward)
    plot(V_vec, fGE_vsV, 'b-', 'LineWidth', 1.5);
    hold on;
    yline(1.0,            'k--', 'No ground effect');
    yline(final_fGE_hover,'b-.', sprintf('Hover fGE %.3f', final_fGE_hover));
    plot(V_ref_forward, fGE_forward_ref, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    hold off;
    xlabel('Forward speed V [m/s]');
else
    % 전진속도 데이터 없으면 호버 단일값만 표시
    yline(final_fGE_hover, 'b-', sprintf('Hover fGE %.3f (V=0)', final_fGE_hover), ...
        'LineWidth', 1.5);
    text(0.5, 0.5, 'forward\_flight\_analysis 미실행 — 속도 스윕 없음', ...
        'Units','normalized', 'HorizontalAlignment','center', 'FontSize', 10);
    xlabel('Forward speed V [m/s]');
end
grid on;
ylabel('Ground effect factor f_{GE}');
title(sprintf('Ground Effect vs Forward Speed at z=%.1f cm', final_height_m*100));

%% Plot 3: 높이 결정 요약 bar chart
figure('Name','Landing Gear Height Decision Summary');
summary_labels = categorical( ...
    {'Final','Gradient Stop','fGE<=Limit','fGE<=1.08','fGE<=1.05'}, ...
    {'Final','Gradient Stop','fGE<=Limit','fGE<=1.08','fGE<=1.05'});
values_cm = [ ...
    final_height_m*100, ...
    gradient_stop_height_m*100, ...
    upper_limit_height_m*100, ...
    target_ref_height_m*100, ...
    stable_height_m*100];
bar(summary_labels, values_cm);
ylabel('Rotor height [cm]');
title(sprintf('Landing Gear Height Decision — %s', prop_name_ge));
grid on;

%% =========================================================================
% 로컬 함수: computeQuadGroundEffect
%
% 쿼드로터 지면효과 모델 (단일 로터 + 로터 간 간섭 + fountain effect)
%
% 입력:
%   z         — 로터 높이 [m], 스칼라 또는 벡터
%   R         — 로터 반경 [m]
%   d         — 인접 로터 축 간 거리 [m]
%   b         — 대각 로터 축 간 거리 [m]
%   Kb        — fountain effect 보정 계수
%   V_over_vi — 전진속도 비 V/vi, 스칼라 또는 벡터
%
% 출력:
%   fGE       — IGE/OGE 추력 비율
%
% 벡터 입력 규칙:
%   z와 V_over_vi 중 하나만 벡터이거나, 둘 다 같은 길이 벡터여야 함.
% =========================================================================
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