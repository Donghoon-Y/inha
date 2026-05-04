%% ========================================================================
% planar_analysis.m
% main.m 실행 후 자동으로 연결되는 planar 분석 통합 코드
%
% [수정] 질량 수렴 반영:
%   2안(forward_flight_analysis.m)이 먼저 실행된 경우
%   mass_Total_ff (수렴 후 2안 질량)를 base_aum으로 사용.
%   1안이거나 mass_Total_ff 가 없으면 mass_Total을 그대로 사용.
%
% 1. 암 길이 계산 (planar_arm_length)
% 2. 추력 증가율 비교 (planar_thrust_compare)
% 3. 발생 추력 vs 필요 추력 비교 (planar_increase_mass)
%% ========================================================================

%% 설계안 분기 및 base_aum 결정
if exist('prop_ff','var') && ~isempty(prop_ff)
    analysis_mode = 'Forward-flight optimum propeller (2안)';
    d             = prop_ff{3}/2;   % 프롭 반경 [inch]

    % ★ 수렴 후 2안 질량 우선 사용, 없으면 mass_Total 폴백
    if exist('mass_Total_ff','var') && ~isempty(mass_Total_ff)
        base_aum = mass_Total_ff;
        mass_src = sprintf('mass_Total_ff (수렴값, 1안 대비 %+.1f g)', ...
                           mass_Total_ff - mass_Total);
    else
        base_aum = mass_Total;
        mass_src = 'mass_Total (2안 수렴값 없음 - forward_flight_analysis 먼저 실행 필요)';
        warning('planar_analysis: mass_Total_ff 없음. mass_Total 사용.');
    end
    twr = ThrustWeightRatio;

else
    analysis_mode = 'Hover optimum propeller (1안)';
    d             = propSpecification{2}/2;
    base_aum      = mass_Total;
    mass_src      = 'mass_Total (1안 실측값)';
    twr           = ThrustWeightRatio;
end

fprintf('\n==============================\n');
fprintf(' Planar 분석 시작\n');
fprintf('==============================\n');
fprintf('설계안          : %s\n', analysis_mode);
fprintf('질량 출처       : %s\n', mass_src);
fprintf('프롭 반경        : %.1f inch\n', d);
fprintf('총 AUM          : %.0f g\n', base_aum);
fprintf('추중비 (TWR)    : %.1f\n', twr);
fprintf('------------------------------\n\n');

%% ========================================================================
%% [1단계] 암 길이 계산
%% ========================================================================
fprintf('=== 1단계: 암 길이 계산 ===\n');

s = 3.0;   % spacing (L/R) - 1번 논문 결론값

arm_length_h = @(x) (((x/25.4)/2) * sqrt(2)) / d;
arm_sol_h    = @(x) arm_length_h(x) - s;

L_total = fzero(arm_sol_h, [0.001, 1000]);
L = L_total / 2;   % 암 길이: 프레임 중심 → 프롭까지

base_arm = 250;    % 기준 암 길이 [mm] (S500 기준)

fprintf('암 길이            : %.0f mm\n', L);
fprintf('대각 프롭사이 거리  : %.0f mm\n', 2*L);
fprintf('늘어난 암 길이      : %+.0f mm  (기준 %d mm 대비)\n', L - base_arm, base_arm);
fprintf('\n');

%% ========================================================================
%% [2단계] 추력 증가율 비교 (spacing별 발생 추력)
%% ========================================================================
fprintf('=== 2단계: spacing별 발생 추력 비교 ===\n');

lr_labels = {'L/R = 2.2', 'L/R = 2.6', 'L/R = 3.0', 'L/R = 3.2', 'L/R = 3.6', 'L/R = 4.0'};

% 논문 Figure 4 추력 증가율 (RPM별 추정값, %)
increment_data = [
    29, 25, 24, 24, 28, 28, 28, 29;  % L/R = 2.2
    30, 26, 25, 26, 26, 27, 29, 29;  % L/R = 2.6
    32, 30, 29, 29, 30, 30, 31, 31;  % L/R = 3.0
    35, 33, 33, 31, 30, 30, 31, 31;  % L/R = 3.2
    36, 31, 30, 21, 22, 31, 31, 32;  % L/R = 3.6
    28, 29, 25, 20, 21, 28, 30, 35;  % L/R = 4.0
];

avg_increment = mean(increment_data, 2);

% base_aum 기준 필요 추력 및 spacing별 발생 추력
base_thrust   = twr * base_aum;                            % 기준 필요 추력 [gf]
thrust_values = base_thrust * (1 + avg_increment / 100);   % spacing별 발생 추력 [gf]

fprintf('%-14s %10s %12s\n', 'L/R', '증가율(%)', '발생 추력(gf)');
fprintf('%s\n', repmat('-', 1, 38));
for i = 1:length(lr_labels)
    fprintf('%-14s %10.1f %12.0f\n', lr_labels{i}, avg_increment(i), thrust_values(i));
end
[~, max_idx] = max(thrust_values);
fprintf('\n최적 spacing: %s (추력 %.0f gf)\n\n', lr_labels{max_idx}, thrust_values(max_idx));

% 그래프
figure('Color', 'white', 'Position', [100, 100, 900, 500]);
bar_colors = repmat([0.52, 0.71, 0.87], length(thrust_values), 1);
bar_colors(max_idx, :) = [0.10, 0.37, 0.65];
hold on;
for i = 1:length(thrust_values)
    b = bar(i, thrust_values(i), 0.6);
    b.FaceColor = bar_colors(i,:); b.EdgeColor = 'none';
end
yline(base_thrust, '--', 'Color', [0.87,0.27,0.27], 'LineWidth', 1.5, ...
    'Label', sprintf('기준 필요 추력 %.0f gf (TWR=%.0f)', base_thrust, twr), ...
    'LabelHorizontalAlignment', 'right', 'FontSize', 11);
set(gca, 'XTick', 1:length(lr_labels), 'XTickLabel', lr_labels, ...
    'FontSize', 12, 'Box', 'off', 'YGrid', 'on', ...
    'GridColor', [0.85,0.85,0.85], 'GridAlpha', 1, 'TickDir', 'out');
ylabel('발생 추력 (gf)', 'FontSize', 13);
title(sprintf('[%s] L/R spacing별 발생 추력\n(TWR=%.0f, AUM=%.0fg)', ...
    analysis_mode, twr, base_aum), 'FontSize', 13, 'FontWeight', 'normal');
h1 = bar(nan, nan, 'FaceColor', [0.52,0.71,0.87], 'EdgeColor', 'none');
h2 = plot(nan, nan, '--', 'Color', [0.87,0.27,0.27], 'LineWidth', 1.5);
legend([h1,h2], {'발생 추력 (gf)', sprintf('기준 필요 추력 (%.0f gf)', base_thrust)}, ...
    'Location', 'northwest', 'FontSize', 11, 'Box', 'off');
hold off;

%% ========================================================================
%% [3단계] 발생 추력 vs 필요 추력 비교 (암 무게 증가 반영)
%% ========================================================================
fprintf('=== 3단계: 발생 추력 vs 필요 추력 비교 (암 무게 반영) ===\n');

n_arms        = 4;
OD            = 16; ID = 14;
rho_carbon    = 1.55e-3;            % 카본 파이프 밀도 [g/mm³]
area_mm2      = pi/4 * (OD^2 - ID^2);
weight_per_mm = area_mm2 * rho_carbon;

spacings = [2.2, 2.6, 3.0, 3.2, 3.6, 4.0];
labels   = {'L/R=2.2','L/R=2.6','L/R=3.0','L/R=3.2','L/R=3.6','L/R=4.0'};

arm_lengths  = spacings * d * 25.4 / sqrt(2);   % spacing별 암 길이 [mm]
delta_arm    = arm_lengths - base_arm;
added_weight = n_arms * delta_arm * weight_per_mm;

% ★ base_aum (수렴 질량 반영) 기준으로 총 AUM 계산
total_aum = base_aum + added_weight;

thrust_gen = twr * base_aum * (1 + avg_increment' / 100);  % 발생 추력 [gf]
thrust_req = twr * total_aum;                               % 필요 추력 [gf]
margin_val = thrust_gen - thrust_req;

fprintf('카본튜브 OD=%dmm/ID=%dmm  단위무게: %.4f g/mm\n', OD, ID, weight_per_mm);
fprintf('base_aum = %.0f g  (암 무게 증분의 기준)\n\n', base_aum);

fprintf('%-10s %9s %9s %9s %10s %6s\n', 'L/R','총AUM(g)','발생추력','필요추력','여유(gf)','달성');
fprintf('%s\n', repmat('-', 1, 60));
for i = 1:length(spacings)
    ok = 'O';
    if margin_val(i) < 0; ok = 'X'; end
    fprintf('%-10s %9.1f %9.1f %9.1f %10.1f %6s\n', ...
        labels{i}, total_aum(i), thrust_gen(i), thrust_req(i), margin_val(i), ok);
end

% ★ 여유 추력이 양수인 spacing 중 최적 출력
valid_mask  = margin_val > 0;
if any(valid_mask)
    [best_margin, best_i] = max(margin_val .* valid_mask);
    fprintf('\n★ 권장 spacing: %s  (여유 추력 +%.0f gf, 총 AUM %.0f g)\n', ...
        labels{best_i}, best_margin, total_aum(best_i));
else
    fprintf('\n[경고] 모든 spacing에서 필요 추력 미달. 프롭 크기 또는 TWR 재검토 필요.\n');
end

% 그래프
figure('Color','white','Position',[100,100,1050,540]);
x = 1:length(spacings);
bar_w = 0.35;
hold on;

for i = 1:length(x)
    fc = [0.10,0.37,0.65];
    if margin_val(i) < 0; fc = [0.95,0.45,0.35]; end
    b1 = bar(x(i)-bar_w/2, thrust_gen(i), bar_w);
    b1.FaceColor = fc; b1.EdgeColor = 'none';
end
for i = 1:length(x)
    b2 = bar(x(i)+bar_w/2, thrust_req(i), bar_w);
    b2.FaceColor = [0.85,0.85,0.85]; b2.EdgeColor = 'none';
end
for i = 1:length(x)
    ypos = max(thrust_gen(i), thrust_req(i)) + 20;
    if margin_val(i) >= 0
        clr = [0.10,0.45,0.15]; txt = sprintf('+%.0f', margin_val(i));
    else
        clr = [0.75,0.10,0.10]; txt = sprintf('%.0f', margin_val(i));
    end
    text(x(i), ypos, txt, 'HorizontalAlignment','center', ...
        'FontSize', 11, 'Color', clr, 'FontWeight', 'bold');
end

set(gca,'XTick',x,'XTickLabel',labels,'FontSize',12,'Box','off','YGrid','on',...
    'GridColor',[0.85,0.85,0.85],'GridAlpha',1,'TickDir','out');
ylabel('추력 / 무게 (gf)', 'FontSize', 13);
ylo = min([thrust_gen(:); thrust_req(:)]) * 0.97;
yhi = max([thrust_gen(:); thrust_req(:)]) * 1.04;
ylim([ylo, yhi]);
title(sprintf('[%s]\nL/R spacing별 발생 추력 vs 필요 추력 (AUM=%.0fg, OD%d/ID%dmm)', ...
    analysis_mode, base_aum, OD, ID), 'FontSize',13,'FontWeight','normal');

h1 = bar(nan,nan,'FaceColor',[0.10,0.37,0.65],'EdgeColor','none');
h2 = bar(nan,nan,'FaceColor',[0.95,0.45,0.35],'EdgeColor','none');
h3 = bar(nan,nan,'FaceColor',[0.85,0.85,0.85],'EdgeColor','none');
legend([h1,h2,h3], {'발생 추력 (TWR 달성)','발생 추력 (TWR 미달)','필요 추력 (TWR×AUM)'}, ...
    'Location','northwest','FontSize',11,'Box','off');
hold off;

fprintf('\n==============================\n');
fprintf(' Planar 분석 완료\n');
fprintf(' 사용 AUM : %.0f g (%s)\n', base_aum, mass_src);
fprintf('==============================\n');