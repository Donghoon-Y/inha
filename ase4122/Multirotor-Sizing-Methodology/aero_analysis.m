%% =========================================================================
% 공력 해석 (Aerodynamic Analysis)
%
% ── 실행 순서 ──────────────────────────────────────────────────────────────
%   main.m 실행 후 이 파일 실행
%   ※ 이 파일은 항상 1안(호버 최적 프롭, temp_propChosen_pos) 기준으로 분석함.
%      2안(기동성 최적 프롭)의 FM/비추력이 필요하면 forward_flight_analysis
%      실행 후 해당 프롭 인덱스(idx_ff_best)로 직접 조회할 것.
%
% ── 분석 항목 ──────────────────────────────────────────────────────────────
%   [1순위] Disk Loading & Figure of Merit
%           → 프롭 후보 전체의 DL·FM·비추력 비교 및 선정 근거 시각화
%
%   ※ [지면효과 & 랜딩기어 높이]는 ground_effect_analysis.m에서 수행.
%      Cheeseman-Bennett 단일 로터 모델보다 로터 간 간섭·fountain effect를
%      포함한 멀티로터 전용 모델(computeQuadGroundEffect)을 사용하므로
%      이 파일에서 중복 구현하지 않음.
%
% ── 참조하는 Workspace 변수 (main.m 출력) ──────────────────────────────────
%   필수:
%     propSpecification    — {name, diam_in, pitch_in}
%     propList_considered  — 후보 프롭 목록 (consideredNo × 6)
%     operatingPoints      — 후보 프롭별 운용점 cell
%     selectionCriterion   — 선정 점수 행렬 (inf = 탈락)
%     temp_propChosen_pos  — 선정 프롭 인덱스 (1안)
%     consideredNo         — 후보 프롭 수
%     thrustHover_Est      — 로터당 호버 추력 [g] (mass_Total_Est 기반 추정치)
% =========================================================================

%% ── [사전 확인] 필요 변수 존재 여부 ─────────────────────────────────────
required_vars = {'propSpecification', 'propList_considered', 'operatingPoints', ...
                 'selectionCriterion', 'temp_propChosen_pos', ...
                 'consideredNo', 'thrustHover_Est'};
missing = {};
for v = required_vars
    if ~exist(v{1}, 'var'), missing{end+1} = v{1}; end
end
if ~isempty(missing)
    error('aero_analysis: 다음 변수가 없습니다. main.m을 먼저 실행하세요.\n  누락: %s', ...
        strjoin(missing, ', '));
end

%% ── [공통] 물리 상수 및 선정 프롭 기본 정보 ─────────────────────────────
rho   = 1.225;   % 공기 밀도 [kg/m³]
g_acc = 9.81;    % 중력가속도 [m/s²]
IN2M  = 0.0254;  % inch → m

prop_name     = propSpecification{1};
prop_diam_in  = propSpecification{2};
prop_pitch_in = propSpecification{3};
R             = (prop_diam_in * IN2M) / 2;   % 프롭 반경 [m]

fprintf('\n[aero_analysis] 분석 대상: %s  (%.0fin × %.0fin)  R=%.4fm\n', ...
    prop_name, prop_diam_in, prop_pitch_in, R);
fprintf('  ※ 항상 1안(호버 최적 프롭, index=%d) 기준\n', temp_propChosen_pos);
fprintf('  ※ 지면효과 분석은 ground_effect_analysis.m 에서 수행\n\n');

%% =========================================================================
% [1순위] Disk Loading & Figure of Merit
% =========================================================================
fprintf('======= Disk Loading & Figure of Merit =======\n');

%% 탈락 프롭 분류
% selectionCriterion = inf  → Speed Limit 초과 탈락
% selectionCriterion = NaN  → 보간 범위 초과 탈락
isElim = any(isinf(selectionCriterion) | isnan(selectionCriterion), 2);

%% 각 프롭의 DL, FM, 비추력 계산
%
% 사용 수식 (Actuator Disk Theory, 호버링 V=0):
%   DL      = T / A                    [g/cm²]
%   v_i     = sqrt(T_N / (2*rho*A))   [m/s]   유도속도
%   P_ideal = T_N * v_i               [W]     이상 유도동력
%   FM      = P_ideal / P_actual       [-]     Figure of Merit
%   비추력  = T / P_actual             [g/W]
%
%   P_actual 기준:
%   operatingPoints{ii,1}(4) — APC .dat 실측 파워 [W]
%   main.m 선정 기준(specThrust_criterion col2) 및
%   arm_wake_analysis(P_hover_mech)와 동일한 값.
%   → 세 파일 간 파워 기준 완전 일치. 선정 프롭 = 비추력 1위.

n  = consideredNo;
DL = nan(n,1);   % Disk Loading [g/cm²]
FM = nan(n,1);   % Figure of Merit [-]
ST = nan(n,1);   % 비추력 [g/W]

fprintf('%-20s %8s %10s %8s %10s  %s\n', ...
    'Propeller','D(in)','DL(g/cm²)','FM','비추력(g/W)','상태');
fprintf('%s\n', repmat('-',1,72));

for ii = 1:n
    d_m   = propList_considered{ii,3} * IN2M;
    A_m2  = pi * (d_m/2)^2;
    DL(ii) = thrustHover_Est / (A_m2 * 1e4);   % [g/cm²]

    if isElim(ii)
        fprintf('%-20s %8.1f %10.4f %8s %10s  X 탈락\n', ...
            propList_considered{ii,1}, propList_considered{ii,3}, DL(ii), '-','-');
        continue
    end

    T_N      = thrustHover_Est * 1e-3 * g_acc;
    vi       = sqrt(T_N / (2 * rho * A_m2));
    P_ideal  = T_N * vi;
    P_actual = operatingPoints{ii,1}(4);   % APC 실측 파워 [W]
    FM(ii)   = P_ideal / P_actual;
    ST(ii)   = thrustHover_Est / P_actual;

    if ii == temp_propChosen_pos, tag = '선정';
    else,                         tag = '통과';
    end
    fprintf('%-20s %8.1f %10.4f %8.4f %10.4f  %s\n', ...
        propList_considered{ii,1}, propList_considered{ii,3}, ...
        DL(ii), FM(ii), ST(ii), tag);
end

%% 선정 프롭 수치 출력
cp = temp_propChosen_pos;
fprintf('\n[선정 프롭 수치 — 1안]\n');
fprintf('  DL     = %.4f g/cm²\n', DL(cp));
fprintf('  FM     = %.4f  (%.1f%%)\n', FM(cp), FM(cp)*100);
fprintf('  비추력 = %.4f g/W  (통과 후보 중 1위)\n', ST(cp));
fprintf('  P_mech = %.3f W  (APC 실측 파워 기준)\n', operatingPoints{cp,1}(4));

%% ── Fig 1: DL vs FM Scatter + 비추력 등고선 ──────────────────────────────
pass_idx = find(~isElim & ~isnan(FM));
elim_idx = find( isElim & ~isnan(DL));

% 탈락 프롭 FM 계산 (operatingPoints 실측 파워 기준)
DL_elim = []; FM_elim_vec = []; nm_elim = {};
for ii = elim_idx'
    d_m  = propList_considered{ii,3} * IN2M;
    A_m2 = pi * (d_m/2)^2;
    P_ii = operatingPoints{ii,1}(4);
    if isnan(P_ii) || P_ii <= 0, continue; end
    T_N  = thrustHover_Est * 1e-3 * g_acc;
    vi   = sqrt(T_N / (2*rho*A_m2));
    DL_elim(end+1)     = DL(ii);
    FM_elim_vec(end+1) = T_N * vi / P_ii;
    nm_elim{end+1}     = [propList_considered{ii,1} ' X'];
end

pass_other = pass_idx(pass_idx ~= cp);

figure('Name',' DL vs FM', 'Position',[50 50 800 550]);
hold on;

h1 = scatter(DL_elim,        FM_elim_vec,   70,  [0.75 0.75 0.75], 'filled');
h2 = scatter(DL(pass_other), FM(pass_other),70,  [0.2 0.5 0.9],   'filled');
h3 = scatter(DL(cp),         FM(cp),        150, [0.9 0.1 0.1],   'filled');

for k = 1:length(DL_elim)
    text(DL_elim(k)+0.03, FM_elim_vec(k), nm_elim{k}, ...
        'FontSize',7, 'Color',[0.5 0.5 0.5]);
end
for ii = pass_other'
    text(DL(ii)+0.03, FM(ii), propList_considered{ii,1}, ...
        'FontSize',7, 'Color',[0.2 0.5 0.9]);
end
text(DL(cp)+0.03, FM(cp), [prop_name], ...
    'FontSize',9, 'FontWeight','bold', 'Color',[0.9 0.1 0.1]);

% 비추력 등고선
%   ST = FM * sqrt(2*rho / DL_SI) * 1000/g_acc
%   DL_SI [N/m²] = DL[g/cm²] * 1e-3 * g_acc * 1e4
DL_cont = linspace(0.3, 1.8, 400);
FM_cont = linspace(0.56, 0.76, 400);
[DL_grid, FM_grid] = meshgrid(DL_cont, FM_cont);
DL_SI_grid = DL_grid * 1e-3 * g_acc * 1e4;
ST_grid    = FM_grid .* sqrt(2*rho ./ DL_SI_grid) * 1000 / g_acc;

cont_levels = [10 11 12 13 14 15 16];
[C, hC] = contour(DL_grid, FM_grid, ST_grid, cont_levels, ...
    'LineColor',[0.75 0.75 0.75], 'LineStyle','--', 'LineWidth',1.0);
clabel(C, hC, cont_levels, 'FontSize',8, 'Color',[0.5 0.5 0.5]);

ST_cp = ST(cp);
[C2, hC2] = contour(DL_grid, FM_grid, ST_grid, [ST_cp ST_cp], ...
    'LineColor',[0.9 0.1 0.1], 'LineStyle','-', 'LineWidth',2.0);
clabel(C2, hC2, 'FontSize',9, 'Color',[0.9 0.1 0.1], 'LabelSpacing',300);

hold off; grid on;
xlim([0.3 1.8]); ylim([0.56 0.76]);
xlabel('Disk Loading  (g/cm²)', 'FontSize',12);
ylabel('Figure of Merit  (FM)', 'FontSize',12);
title(' Disk Loading vs Figure of Merit + 비추력 등고선 (g/W)', 'FontSize',13);
legend([h1 h2 h3], {'탈락 (Speed Limit 초과)', '통과 후보', '선정 프롭'}, ...
    'Location','northeast', 'FontSize',10);
annotation('textbox',[0.13 0.13 0.38 0.11], ...
    'String', {'회색 점선: 비추력 등고선 (g/W)', ...
               sprintf('빨간 실선: 선정 프롭 비추력 기준선 (%.2f g/W)', ST_cp)}, ...
    'FitBoxToText','on','BackgroundColor',[1 1 0.9], ...
    'FontSize',8,'EdgeColor',[0.7 0.7 0]);

%% ── Fig 2: 비추력 막대그래프 (통과 프롭, 내림차순) ──────────────────────
st_pass     = ST(pass_idx);
nm_pass     = propList_considered(pass_idx, 1);
[st_s, si]  = sort(st_pass, 'descend');
nm_s        = nm_pass(si);
ch_pos      = find(pass_idx(si) == cp);

clr = repmat([0.2 0.5 0.9], length(st_s), 1);
clr(ch_pos,:) = [0.9 0.1 0.1];

figure('Name',' 비추력 비교', 'Position',[50 50 900 500]);
b = bar(st_s); b.FaceColor='flat'; b.CData=clr;
set(gca,'XTick',1:length(st_s),'XTickLabel',nm_s, ...
    'XTickLabelRotation',40,'FontSize',9);
for k = 1:length(st_s)
    text(k, st_s(k)+0.1, sprintf('%.2f',st_s(k)), ...
        'HorizontalAlignment','center','FontSize',8);
end
ylabel('비추력  (g/W)', 'FontSize',12);
title('호버링 비추력 비교 — 통과 프롭 (내림차순)', 'FontSize',13);
ylim([0 max(st_s)*1.15]); grid on;

%% =========================================================================
% 최종 요약
% =========================================================================
fprintf('\n============= 공력 해석 요약 (1안 기준) =============\n');
fprintf('선정 프롭: %s\n', prop_name);
fprintf('  DL     = %.4f g/cm²\n', DL(cp));
fprintf('  FM     = %.4f  (%.1f%%)\n', FM(cp), FM(cp)*100);
fprintf('  비추력 = %.4f g/W  (통과 후보 중 1위)\n', ST(cp));
fprintf('\n지면효과 & 랜딩기어 높이 설계:\n');
fprintf('  → ground_effect_analysis.m 을 실행하세요.\n');
fprintf('     (멀티로터 전용 모델, 로터 간 간섭·fountain effect 포함)\n');
fprintf('=====================================================\n\n');