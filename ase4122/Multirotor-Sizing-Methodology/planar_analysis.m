%% ========================================================================
% planar_analysis_streamline.m
% 실행 순서: main.m → arm_wake_analysis.m → (forward_flight_analysis.m) → 이 파일
%
% [통합 내용]
%   구 planar_analysis.m  : 암 길이 계산 + L/R spacing별 추력 비교
%   구 inverted_teardrop_mass.m : 렘니스케이트 단면 기반 암 질량 계산
%
% [암 단면 모델]
%   US11305881 특허 렘니스케이트(역물방울) 단면
%   (y²+x²)² = 2a²(y²-x²), 상단 루프
%   외곽: a_outer = h / √2,  폭 w = a_outer
%   내부: a_inner = a_outer - tw
%   단면적: A = a²  (렘니스케이트 루프 면적)
%
% [분석 범위]
%   1단계: 렘니스케이트 단면 치수 계산
%   2단계: 암 길이 계산 (L/R = 3.0 기준)
%   3단계: Planar — L/R spacing별 추력 여유 비교
%   4단계: Non-Planar — l/d spacing별 추력 여유 비교
%
% [분기]
%   prop_ff 변수 존재 → 2안(전진비행 최적 프롭)
%   없음              → 1안(호버 최적 프롭)
%   mass_Total_ff 존재 → 수렴 후 2안 질량 우선 사용
%% ========================================================================

%% ── 실행 순서 확인 ───────────────────────────────────────────────────────
if ~exist('propSpecification','var') || ~exist('mass_Total','var')
    error('main.m을 먼저 실행하세요.');
end
if ~exist('bestPracticalCandidate','var')
    error('arm_wake_analysis.m을 먼저 실행하세요. (렘니스케이트 단면 치수 필요)');
end

%% ── 설계안 분기 및 base_aum 결정 ─────────────────────────────────────────
if exist('prop_ff','var') && ~isempty(prop_ff)
    analysis_mode = 'Forward-flight optimum propeller (2안)';
    d_prop        = prop_ff{3} / 2;          % 프롭 반경 [inch]

    if exist('mass_Total_ff','var') && ~isempty(mass_Total_ff)
        base_aum = mass_Total_ff;
        mass_src = sprintf('mass_Total_ff (수렴값, 1안 대비 %+.1f g)', ...
                           mass_Total_ff - mass_Total);
    else
        base_aum = mass_Total;
        mass_src = 'mass_Total (2안 수렴값 없음 — forward_flight_analysis 먼저 실행 필요)';
        warning('planar_analysis_streamline: mass_Total_ff 없음. mass_Total 사용.');
    end
else
    analysis_mode = 'Hover optimum propeller (1안)';
    d_prop        = propSpecification{2} / 2; % 프롭 반경 [inch]
    base_aum      = mass_Total;
    mass_src      = 'mass_Total (1안 실측값)';
end

twr = ThrustWeightRatio;

fprintf('\n==============================\n');
fprintf(' Planar/Non-Planar 통합 분석 시작\n');
fprintf(' (렘니스케이트 단면 기반)\n');
fprintf('==============================\n');
fprintf('설계안          : %s\n', analysis_mode);
fprintf('질량 출처       : %s\n', mass_src);
fprintf('프롭 반경        : %.2f inch\n', d_prop);
fprintf('총 AUM          : %.0f g\n', base_aum);
fprintf('추중비 (TWR)    : %.1f\n', twr);
fprintf('------------------------------\n\n');

%% ========================================================================
%% [1단계] 렘니스케이트 단면 치수 계산
%% ========================================================================
fprintf('=== 1단계: 렘니스케이트(역물방울) 단면 치수 ===\n');

%── arm_wake_analysis 결과 수신
h_mm           = bestPracticalCandidate.ArmThickness_mm;   % 유동 방향 투영 높이 [mm]
arm_thrustloss = bestPracticalCandidate.ThrustLoss_pct;    % 추력 손실 [%]

%── 렘니스케이트 치수
tw           = 1.0;              % 벽 두께 [mm]
rho_carbon   = 1.55e-3;          % 카본 밀도 [g/mm³]

a_outer      = h_mm / sqrt(2);   % 외곽 a
a_inner      = a_outer - tw;     % 내부 a (벽 두께 반영)
w_mm         = a_outer;          % 최대 폭 (= a_outer)

A_outer      = a_outer^2;        % 외곽 루프 단면적 [mm²]
A_inner      = a_inner^2;        % 내부 루프 단면적 [mm²]
A_shell      = A_outer - A_inner; % 쉘(벽) 단면적 [mm²]
weight_per_mm = A_shell * rho_carbon;          % 단위 길이당 무게 [g/mm]
thrust_loss_factor = 1 - arm_thrustloss / 100; % 추력 보정 계수

fprintf('벽 두께 (tw)       : %.1f mm\n',   tw);
fprintf('유동 방향 높이 (h) : %.2f mm\n',   h_mm);
fprintf('a 외곽             : %.2f mm\n',   a_outer);
fprintf('최대 폭 (w)        : %.2f mm\n',   w_mm);
fprintf('h/w 비율           : %.3f  (이론값 √2 = %.3f)\n', h_mm/w_mm, sqrt(2));
fprintf('외곽 단면적        : %.3f mm²\n',  A_outer);
fprintf('내부 단면적        : %.3f mm²\n',  A_inner);
fprintf('쉘 단면적          : %.3f mm²\n',  A_shell);
fprintf('단위 무게          : %.4f g/mm\n', weight_per_mm);
fprintf('예상 추력 손실     : %.3f %%\n',   arm_thrustloss);
fprintf('------------------------------\n\n');

%% ========================================================================
%% [2단계] 암 길이 계산 (L/R = 3.0 기준)
%% ========================================================================
fprintf('=== 2단계: 암 길이 계산 (L/R = 3.0) ===\n');

s_ref    = 3.0;   % 기준 spacing (논문 결론값)
base_arm = 250;   % 기준 암 길이 [mm] (S500 기준)
n_arms   = 4;

%── 암 길이 계산 함수: L/R = (2L/25.4) / (2·d_prop) → L = s·d_prop·25.4/√2 / 2
arm_fn = @(x) (((x / 25.4) / 2) * sqrt(2)) / d_prop;
L_ref  = fzero(@(x) arm_fn(x) - s_ref, [0.001, 1000]) / 2;

fprintf('암 길이 (중심→프롭) : %.0f mm\n', L_ref);
fprintf('대각 프롭 간 거리   : %.0f mm\n', 2 * L_ref);
fprintf('늘어난 암 길이      : %+.0f mm  (기준 %d mm 대비)\n', L_ref - base_arm, base_arm);
fprintf('------------------------------\n\n');

%% ========================================================================
%% [3단계] Planar — L/R spacing별 추력 여유 비교
%% ========================================================================
fprintf('=== 3단계: Planar — L/R spacing별 추력 여유 ===\n');

spacings_p = [2.2, 2.6, 3.0, 3.2, 3.6, 4.0];
lr_labels  = {'L/R=2.2','L/R=2.6','L/R=3.0','L/R=3.2','L/R=3.6','L/R=4.0'};

%── 논문 Figure 4 추력 증가율 [%] (RPM 8구간 평균)
increment_data_p = [
    29, 25, 24, 24, 28, 28, 28, 29;   % L/R = 2.2
    30, 26, 25, 26, 26, 27, 29, 29;   % L/R = 2.6
    32, 30, 29, 29, 30, 30, 31, 31;   % L/R = 3.0
    35, 33, 33, 31, 30, 30, 31, 31;   % L/R = 3.2
    36, 31, 30, 21, 22, 31, 31, 32;   % L/R = 3.6
    28, 29, 25, 20, 21, 28, 30, 35;   % L/R = 4.0
];
avg_inc_p = mean(increment_data_p, 2);  % spacing별 평균 증가율

%── 렘니스케이트 단면 기반 암 무게 증분
arm_len_p    = spacings_p * d_prop * 25.4 / sqrt(2);   % spacing별 암 길이 [mm]
delta_arm_p  = arm_len_p - base_arm;                    % 기준 대비 증분 [mm]
add_wt_p     = n_arms * delta_arm_p * weight_per_mm;    % 추가 암 무게 [g]

%── 추력 계산 (추력 손실 계수 반영)
total_aum_p  = base_aum + add_wt_p;                              % 총 AUM [g]
thrust_gen_p = twr * base_aum * (1 + avg_inc_p'/100) ...
               * thrust_loss_factor;                              % 발생 추력 [gf]
thrust_req_p = twr * total_aum_p;                                % 필요 추력 [gf]
margin_p     = thrust_gen_p - thrust_req_p;                      % 여유 추력 [gf]

fprintf('렘니스케이트 단위무게: %.4f g/mm  (tw=%.0fmm, h=%.1fmm)\n', ...
        weight_per_mm, tw, h_mm);
fprintf('base_aum: %.0f g\n\n', base_aum);

fprintf('%-10s %9s %9s %9s %10s %6s\n', 'L/R','총AUM(g)','발생추력','필요추력','여유(gf)','달성');
fprintf('%s\n', repmat('-', 1, 60));
for i = 1:length(spacings_p)
    ok = 'O'; if margin_p(i) < 0; ok = 'X'; end
    fprintf('%-10s %9.1f %9.1f %9.1f %10.1f %6s\n', ...
        lr_labels{i}, total_aum_p(i), thrust_gen_p(i), thrust_req_p(i), margin_p(i), ok);
end

valid_mask_p = margin_p > 0;
if any(valid_mask_p)
    [best_margin_p, best_i_p] = max(margin_p .* valid_mask_p);
    fprintf('\n★ 권장 spacing: %s  (여유 추력 +%.0f gf, 총 AUM %.0f g)\n', ...
        lr_labels{best_i_p}, best_margin_p, total_aum_p(best_i_p));
else
    fprintf('\n[경고] 모든 Planar spacing에서 필요 추력 미달. 프롭 크기 또는 TWR 재검토 필요.\n');
end
fprintf('\n');

%── Planar 그래프
figure('Color','white','Position',[100,100,1050,520]);
x = 1:length(spacings_p); bar_w = 0.35;
hold on;
for i = 1:length(x)
    fc = [0.10,0.37,0.65]; if margin_p(i) < 0; fc = [0.95,0.45,0.35]; end
    b1 = bar(x(i)-bar_w/2, thrust_gen_p(i), bar_w);
    b1.FaceColor = fc; b1.EdgeColor = 'none';
end
for i = 1:length(x)
    b2 = bar(x(i)+bar_w/2, thrust_req_p(i), bar_w);
    b2.FaceColor = [0.85,0.85,0.85]; b2.EdgeColor = 'none';
end
for i = 1:length(x)
    ypos = max(thrust_gen_p(i), thrust_req_p(i)) + 20;
    if margin_p(i) >= 0
        clr = [0.10,0.45,0.15]; txt = sprintf('+%.0f', margin_p(i));
    else
        clr = [0.75,0.10,0.10]; txt = sprintf('%.0f',  margin_p(i));
    end
    text(x(i), ypos, txt, 'HorizontalAlignment','center', ...
         'FontSize',11, 'Color',clr, 'FontWeight','bold');
end
set(gca,'XTick',x,'XTickLabel',lr_labels,'FontSize',12,'Box','off','YGrid','on', ...
    'GridColor',[0.85,0.85,0.85],'GridAlpha',1,'TickDir','out');
ylabel('추력 (gf)', 'FontSize',13);
ylim([min([thrust_gen_p(:);thrust_req_p(:)])*0.97, ...
      max([thrust_gen_p(:);thrust_req_p(:)])*1.04]);
title(sprintf('[Planar | %s]\nL/R spacing별 발생 vs 필요 추력 — 렘니스케이트 arm h=%.1fmm, tw=%.0fmm (TWR=%.0f, AUM=%.0fg)', ...
    analysis_mode, h_mm, tw, twr, base_aum), 'FontSize',12,'FontWeight','normal');
h1 = bar(nan,nan,'FaceColor',[0.10,0.37,0.65],'EdgeColor','none');
h2 = bar(nan,nan,'FaceColor',[0.95,0.45,0.35],'EdgeColor','none');
h3 = bar(nan,nan,'FaceColor',[0.85,0.85,0.85],'EdgeColor','none');
legend([h1,h2,h3], {'발생 추력 (TWR 달성)','발생 추력 (TWR 미달)','필요 추력 (TWR×AUM)'}, ...
    'Location','northwest','FontSize',11,'Box','off');
hold off;

%% ========================================================================
%% [4단계] Non-Planar — l/d spacing별 추력 여유 비교 (tilt = 30°)
%% ========================================================================
fprintf('=== 4단계: Non-Planar — l/d spacing별 추력 여유 (tilt=30°) ===\n');

ld_sp   = [1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0];
ld_lbl  = arrayfun(@(x) sprintf('l/d=%.1f',x), ld_sp, 'UniformOutput', false);

%── Non-Planar 추력 증가율 [%] (l/d별 추정값)
avg_inc_ld = [5.5, 4.8, 4.2, 4.0, 4.0, 4.2, 3.8, 4.5, 3.5, 3.2];

%── l/d → 등가 L/R = 2·(l/d) 로 환산하여 암 길이 계산
lr_nonp     = 2 * ld_sp;
arm_len_n   = lr_nonp * d_prop * 25.4 / sqrt(2);
delta_arm_n = arm_len_n - base_arm;
add_wt_n    = n_arms * delta_arm_n * weight_per_mm;

total_aum_n  = base_aum + add_wt_n;
thrust_gen_n = twr * base_aum * (1 + avg_inc_ld/100) * thrust_loss_factor;
thrust_req_n = twr * total_aum_n;
margin_n     = thrust_gen_n - thrust_req_n;

fprintf('base_aum: %.0f g\n\n', base_aum);

fprintf('%-10s %9s %9s %9s %10s %6s\n', 'l/d','총AUM(g)','발생추력','필요추력','여유(gf)','달성');
fprintf('%s\n', repmat('-', 1, 60));
for i = 1:length(ld_sp)
    ok = 'O'; if margin_n(i) < 0; ok = 'X'; end
    fprintf('%-10s %9.1f %9.1f %9.1f %10.1f %6s\n', ...
        ld_lbl{i}, total_aum_n(i), thrust_gen_n(i), thrust_req_n(i), margin_n(i), ok);
end

valid_mask_n = margin_n > 0;
if any(valid_mask_n)
    [best_margin_n, best_i_n] = max(margin_n .* valid_mask_n);
    fprintf('\n★ 권장 l/d: %s  (여유 추력 +%.0f gf, 총 AUM %.0f g)\n', ...
        ld_lbl{best_i_n}, best_margin_n, total_aum_n(best_i_n));
else
    fprintf('\n[경고] 모든 Non-Planar spacing에서 필요 추력 미달. 프롭 크기 또는 TWR 재검토 필요.\n');
end

%── Non-Planar 그래프
figure('Color','white','Position',[100,100,1100,520]);
x = 1:length(ld_sp); bar_w = 0.35;
hold on;
for i = 1:length(x)
    fc = [0.10,0.37,0.65]; if margin_n(i) < 0; fc = [0.95,0.45,0.35]; end
    b1 = bar(x(i)-bar_w/2, thrust_gen_n(i), bar_w);
    b1.FaceColor = fc; b1.EdgeColor = 'none';
end
for i = 1:length(x)
    b2 = bar(x(i)+bar_w/2, thrust_req_n(i), bar_w);
    b2.FaceColor = [0.85,0.85,0.85]; b2.EdgeColor = 'none';
end
xline(find(ld_sp == 1.4), '--', 'Color',[0.87,0.55,0.10], 'LineWidth',1.5, ...
    'Label','l/d=1.4 기준', 'LabelHorizontalAlignment','right', 'FontSize',10);
for i = 1:length(x)
    ypos = max(thrust_gen_n(i), thrust_req_n(i)) + 15;
    if margin_n(i) >= 0
        clr = [0.10,0.45,0.15]; txt = sprintf('+%.0f', margin_n(i));
    else
        clr = [0.75,0.10,0.10]; txt = sprintf('%.0f',  margin_n(i));
    end
    text(x(i), ypos, txt, 'HorizontalAlignment','center', ...
         'FontSize',10, 'Color',clr, 'FontWeight','bold');
end
set(gca,'XTick',x,'XTickLabel',ld_lbl,'FontSize',12,'Box','off','YGrid','on', ...
    'GridColor',[0.85,0.85,0.85],'GridAlpha',1,'TickDir','out');
ylabel('추력 (gf)', 'FontSize',13);
ylim([min([thrust_gen_n(:);thrust_req_n(:)])*0.97, ...
      max([thrust_gen_n(:);thrust_req_n(:)])*1.04]);
title(sprintf('[Non-Planar | %s]\nl/d spacing별 발생 vs 필요 추력 — 렘니스케이트 arm h=%.1fmm, tw=%.0fmm, tilt=30° (TWR=%.0f, AUM=%.0fg)', ...
    analysis_mode, h_mm, tw, twr, base_aum), 'FontSize',12,'FontWeight','normal');
h1 = bar(nan,nan,'FaceColor',[0.10,0.37,0.65],'EdgeColor','none');
h2 = bar(nan,nan,'FaceColor',[0.95,0.45,0.35],'EdgeColor','none');
h3 = bar(nan,nan,'FaceColor',[0.85,0.85,0.85],'EdgeColor','none');
legend([h1,h2,h3], {'발생 추력 (TWR 달성)','발생 추력 (TWR 미달)','필요 추력 (TWR×AUM)'}, ...
    'Location','northwest','FontSize',11,'Box','off');
hold off;

%% ========================================================================
%% [5단계] 최종 확정 무게 출력 (권장 spacing 기준)
%% ========================================================================
fprintf('=== 5단계: 최종 확정 무게 (권장 spacing 기준) ===\n\n');

%── Planar 최종 확정 무게
if exist('best_i_p','var')
    best_arm_len_p   = arm_len_p(best_i_p);
    best_arm_wt_p    = n_arms * best_arm_len_p * weight_per_mm;
    best_arm_delta_p = add_wt_p(best_i_p);
    final_aum_p      = total_aum_p(best_i_p);

    fprintf('[Planar 최종 확정]\n');
    fprintf('  권장 spacing      : %s\n',      lr_labels{best_i_p});
    fprintf('  암 길이 (1개)     : %.1f mm\n', best_arm_len_p);
    fprintf('  암 4개 총 무게    : %.1f g   (%.4f g/mm × %.0f mm × 4)\n', ...
            best_arm_wt_p, weight_per_mm, best_arm_len_p);
    fprintf('  암 무게 증분      : %+.1f g  (기준 암 대비)\n', best_arm_delta_p);
    fprintf('  ┌─────────────────────────────────────┐\n');
    fprintf('  │  기본 AUM (전장품)  : %8.1f g    │\n', base_aum);
    fprintf('  │  + 암 무게 증분     : %+8.1f g    │\n', best_arm_delta_p);
    fprintf('  │  ─────────────────────────────────  │\n');
    fprintf('  │  최종 AUM (Planar)  : %8.1f g    │\n', final_aum_p);
    fprintf('  └─────────────────────────────────────┘\n\n');
else
    fprintf('[경고] Planar 권장 spacing 없음 — 최종 무게 산출 불가\n\n');
end

%── Non-Planar 최종 확정 무게
if exist('best_i_n','var')
    best_arm_len_n   = arm_len_n(best_i_n);
    best_arm_wt_n    = n_arms * best_arm_len_n * weight_per_mm;
    best_arm_delta_n = add_wt_n(best_i_n);
    final_aum_n      = total_aum_n(best_i_n);

    fprintf('[Non-Planar 최종 확정]\n');
    fprintf('  권장 l/d          : %s\n',      ld_lbl{best_i_n});
    fprintf('  암 길이 (1개)     : %.1f mm\n', best_arm_len_n);
    fprintf('  암 4개 총 무게    : %.1f g   (%.4f g/mm × %.0f mm × 4)\n', ...
            best_arm_wt_n, weight_per_mm, best_arm_len_n);
    fprintf('  암 무게 증분      : %+.1f g  (기준 암 대비)\n', best_arm_delta_n);
    fprintf('  ┌─────────────────────────────────────┐\n');
    fprintf('  │  기본 AUM (전장품)  : %8.1f g    │\n', base_aum);
    fprintf('  │  + 암 무게 증분     : %+8.1f g    │\n', best_arm_delta_n);
    fprintf('  │  ─────────────────────────────────  │\n');
    fprintf('  │  최종 AUM (Non-P)   : %8.1f g    │\n', final_aum_n);
    fprintf('  └─────────────────────────────────────┘\n\n');
else
    fprintf('[경고] Non-Planar 권장 l/d 없음 — 최종 무게 산출 불가\n\n');
end

%% ========================================================================
%% 최종 요약
%% ========================================================================
fprintf('==============================\n');
fprintf(' 통합 분석 완료\n');
fprintf('==============================\n');
fprintf('설계안      : %s\n', analysis_mode);
fprintf('질량 출처   : %s\n', mass_src);
fprintf('암 단면     : 렘니스케이트  h=%.2fmm, w=%.2fmm, tw=%.0fmm\n', h_mm, w_mm, tw);
fprintf('단위 암무게 : %.4f g/mm\n', weight_per_mm);
fprintf('추력 손실   : %.3f %%\n', arm_thrustloss);
fprintf('------------------------------\n');
if exist('best_i_p','var')
    fprintf('[Planar]     권장 %s  →  최종 AUM %7.1f g  (여유 +%.0f gf)\n', ...
        lr_labels{best_i_p}, final_aum_p, best_margin_p);
end
if exist('best_i_n','var')
    fprintf('[Non-Planar] 권장 %s  →  최종 AUM %7.1f g  (여유 +%.0f gf)\n', ...
        ld_lbl{best_i_n}, final_aum_n, best_margin_n);
end
fprintf('==============================\n');