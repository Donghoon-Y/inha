%% =========================================================================
% 공력 해석 (Aerodynamic Analysis)
% 실행 순서: main.m 먼저 실행 후 이 파일 실행
%
% 1순위: Disk Loading & Figure of Merit
% 3순위: 지면효과 & 랜딩기어 높이 설계
% =========================================================================

%% -------------------------------------------------------------------------
% [사전 확인] main.m 실행 여부 체크
% -------------------------------------------------------------------------
if ~exist('propSpecification','var') || ~exist('operatingPoints','var') || ...
   ~exist('selectionCriterion','var') || ~exist('temp_propChosen_pos','var')
    error('main.m을 먼저 실행하세요.');
end

%% -------------------------------------------------------------------------
% [공통] 물리 상수 및 선정 프롭 기본 정보
% -------------------------------------------------------------------------
rho   = 1.225;    % 공기 밀도 [kg/m³]
g_acc = 9.81;     % 중력가속도 [m/s²]
IN2M  = 0.0254;   % inch → m

prop_name     = propSpecification{1};
prop_diam_in  = propSpecification{2};
prop_pitch_in = propSpecification{3};
R             = (prop_diam_in * IN2M) / 2;   % 프롭 반경 [m]

fprintf('\n선정 프롭: %s  (%.0fin × %.0fin pitch)  R=%.4fm\n', ...
    prop_name, prop_diam_in, prop_pitch_in, R);

%% =========================================================================
% [1순위] Disk Loading & Figure of Merit
% =========================================================================
fprintf('\n======= [1순위] Disk Loading & Figure of Merit =======\n');

%% 탈락 프롭 분류
% selectionCriterion = inf  → Speed Limit 초과 탈락
% selectionCriterion = NaN  → 보간 범위 초과 탈락
isElim = any(isinf(selectionCriterion) | isnan(selectionCriterion), 2);

%% 각 프롭의 DL, FM, 비추력 계산
% 사용 수식 (Actuator Disk Theory, 호버링 V=0):
%   DL      = T / A                    [g/cm²]
%   v_i     = sqrt(T_N / (2*rho*A))   [m/s]   유도속도
%   P_ideal = T_N * v_i               [W]     이상 유도동력
%   FM      = P_ideal / P_actual       [-]     Figure of Merit
%   비추력  = T / P_actual             [g/W]
%
% P_actual = mean(selectionCriterion(ii,:))
%   → main.m의 선정 기준(specThrust_criterion)과 동일한 파워 기준
%   → 탈락 프롭은 inf이므로 아래에서 별도 처리

n    = consideredNo;
DL   = nan(n,1);   % Disk Loading [g/cm²]
FM   = nan(n,1);   % Figure of Merit [-]
ST   = nan(n,1);   % 비추력 [g/W]

fprintf('%-20s %8s %10s %8s %10s  %s\n', ...
    'Propeller','D(in)','DL(g/cm²)','FM','비추력(g/W)','상태');
fprintf('%s\n', repmat('-',1,72));

for ii = 1:n
    % 직경 → 면적 (탈락/통과 무관하게 DL은 항상 계산)
    d_m   = propList_considered{ii,3} * IN2M;
    A_m2  = pi * (d_m/2)^2;
    DL(ii) = thrustHover_Est / (A_m2 * 1e4);   % [g/cm²]

    if isElim(ii)
        fprintf('%-20s %8.1f %10.4f %8s %10s  X 탈락\n', ...
            propList_considered{ii,1}, propList_considered{ii,3}, DL(ii), '-','-');
        continue   % FM, ST는 NaN 유지
    end

    % 통과 프롭만 FM, 비추력 계산
    T_N        = thrustHover_Est * 1e-3 * g_acc;
    vi         = sqrt(T_N / (2 * rho * A_m2));
    P_ideal    = T_N * vi;
    P_actual   = mean(selectionCriterion(ii,:));
    FM(ii)     = P_ideal / P_actual;
    ST(ii)     = thrustHover_Est / P_actual;

    if ii == temp_propChosen_pos, tag = '★ 선정';
    else,                         tag = '통과';
    end
    fprintf('%-20s %8.1f %10.4f %8.4f %10.4f  %s\n', ...
        propList_considered{ii,1}, propList_considered{ii,3}, ...
        DL(ii), FM(ii), ST(ii), tag);
end

%% 선정 프롭 수치 출력
cp = temp_propChosen_pos;   % 선정 프롭 인덱스
fprintf('\n[선정 프롭 수치]\n');
fprintf('  DL     = %.4f g/cm²\n', DL(cp));
fprintf('  FM     = %.4f  (%.1f%%)\n', FM(cp), FM(cp)*100);
fprintf('  비추력 = %.4f g/W  (통과 후보 중 1위)\n', ST(cp));

%% Fig 1: DL vs FM Scatter
pass_idx = find(~isElim & ~isnan(FM));
elim_idx = find( isElim & ~isnan(DL));

% 탈락 프롭 FM 계산 (APC 실측 파워 기준)
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

% 통과 프롭 (선정 제외)
pass_other = pass_idx(pass_idx ~= cp);

figure('Name','[1순위] DL vs FM', 'Position',[50 50 800 550]);
hold on;

% 레전드를 위한 핵심 scatter (먼저 그림)
h1 = scatter(DL_elim,      FM_elim_vec,  70,  [0.75 0.75 0.75], 'filled');
h2 = scatter(DL(pass_other), FM(pass_other), 70, [0.2 0.5 0.9],  'filled');
h3 = scatter(DL(cp),       FM(cp),       150, [0.9 0.1 0.1],    'filled');

% 라벨 텍스트
for k = 1:length(DL_elim)
    text(DL_elim(k)+0.03, FM_elim_vec(k), nm_elim{k}, ...
        'FontSize',7, 'Color',[0.5 0.5 0.5]);
end
for ii = pass_other'
    text(DL(ii)+0.03, FM(ii), propList_considered{ii,1}, ...
        'FontSize',7, 'Color',[0.2 0.5 0.9]);
end
text(DL(cp)+0.03, FM(cp), ['★ ' prop_name], ...
    'FontSize',9, 'FontWeight','bold', 'Color',[0.9 0.1 0.1]);

% -----------------------------------------------------------------------
% 비추력 등고선 추가
%   수식: ST = FM * sqrt(2*rho / DL_SI) * 1000/g_acc
%   DL_SI [N/m²] = DL[g/cm²] * 1e-3 * g_acc * 1e4
%
%   물리적 의미:
%   같은 비추력(등고선)의 점들은 DL이 높아도 FM이 높으면 보완 가능
%   선정 프롭은 낮은 DL 덕분에 FM이 낮아도 최고 비추력 달성
% -----------------------------------------------------------------------
% 데이터가 몰린 범위에 맞게 그리드 설정
DL_cont = linspace(0.3, 1.8, 400);
FM_cont = linspace(0.56, 0.76, 400);
[DL_grid, FM_grid] = meshgrid(DL_cont, FM_cont);

DL_SI_grid = DL_grid * 1e-3 * g_acc * 1e4;
ST_grid    = FM_grid .* sqrt(2*rho ./ DL_SI_grid) * 1000 / g_acc;

% 등고선 레벨 — 선정 프롭 비추력 포함
cont_levels = [10 11 12 13 14 15 16];

[C, hC] = contour(DL_grid, FM_grid, ST_grid, cont_levels, ...
    'LineColor',[0.75 0.75 0.75], 'LineStyle','--', 'LineWidth',1.0);
clabel(C, hC, cont_levels, 'FontSize',8, 'Color',[0.5 0.5 0.5]);

% 선정 프롭 비추력 등고선 강조 (빨간 실선)
ST_cp = ST(cp);
[C2, hC2] = contour(DL_grid, FM_grid, ST_grid, [ST_cp ST_cp], ...
    'LineColor',[0.9 0.1 0.1], 'LineStyle','-', 'LineWidth',2.0);
clabel(C2, hC2, 'FontSize',9, 'Color',[0.9 0.1 0.1], ...
    'LabelSpacing',300);

hold off; grid on;
% x축을 데이터 범위에 맞게 제한
xlim([0.3 1.8]);
ylim([0.56 0.76]);
xlabel('Disk Loading  (g/cm²)', 'FontSize',12);
ylabel('Figure of Merit  (FM)', 'FontSize',12);
title('[1순위] Disk Loading vs Figure of Merit + 비추력 등고선 (g/W)', 'FontSize',13);
legend([h1 h2 h3], {'탈락 (Speed Limit 초과)', '통과 후보', '선정 프롭'}, ...
    'Location','northeast', 'FontSize',10);
annotation('textbox',[0.13 0.13 0.38 0.11], ...
    'String', {'회색 점선: 비추력 등고선 (g/W)', ...
               '빨간 실선: 선정 프롭 비추력 기준선 (14.77 g/W)'}, ...
    'FitBoxToText','on','BackgroundColor',[1 1 0.9], ...
    'FontSize',8,'EdgeColor',[0.7 0.7 0]);

%% Fig 2: 비추력 막대그래프 (통과 프롭, 내림차순)
st_pass  = ST(pass_idx);
nm_pass  = propList_considered(pass_idx, 1);
[st_s, si] = sort(st_pass, 'descend');
nm_s     = nm_pass(si);
ch_pos   = find(pass_idx(si) == cp);   % 선정 프롭 위치

clr = repmat([0.2 0.5 0.9], length(st_s), 1);
clr(ch_pos,:) = [0.9 0.1 0.1];

figure('Name','[1순위] 비추력 비교', 'Position',[50 50 900 500]);
b = bar(st_s); b.FaceColor='flat'; b.CData=clr;
set(gca,'XTick',1:length(st_s),'XTickLabel',nm_s, ...
    'XTickLabelRotation',40,'FontSize',9);
for k = 1:length(st_s)
    text(k, st_s(k)+0.1, sprintf('%.2f',st_s(k)), ...
        'HorizontalAlignment','center','FontSize',8);
end
ylabel('비추력  (g/W)', 'FontSize',12);
title('[1순위] 호버링 비추력 비교 — 통과 프롭 (내림차순)', 'FontSize',13);
ylim([0 max(st_s)*1.15]); grid on;

%% =========================================================================
% [3순위] 지면효과 & 랜딩기어 높이 설계
% =========================================================================
fprintf('\n======= [3순위] 지면효과 & 랜딩기어 높이 =======\n');

%% 기하학적 파라미터
% 드론 구조 (착지 시):
%
%   지면
%    ↕  h_lg  (랜딩기어 높이 = 설계 변수)
%   랜딩기어 발끝 = 드론 하단
%    ↕  h_cg  (드론 하단 ~ 프롭 회전면 거리)
%   드론 중심 = 프롭 회전면
%    ↕  R     (프롭 반경, 아래 방향)
%   프롭 끝단
%
%   프롭 끝단 지상고 = h_lg + h_cg - R  > 0 이어야 함
%   → h_lg > R - h_cg
%   안전 여유 50mm 추가: h_safety = R - h_cg + 0.05

h_cg = 0.10;   % 드론 두께 절반 [m]

%% Cheeseman-Bennett 모델
% T_IGE/T_OGE = 1 / (1 - (R/(4z))^2)
%
% z  : 드론 중심(프롭 회전면) 기준 지상고 [m]
% z/R < 0.25 : 수식 특이점 (프롭이 지면에 닿는 영역)
% z/R < 0.5  : 재순환 와류 심각 → PID 제어 불안정
% z/R > 1.5  : 지면효과 소멸

z_vec    = linspace(0.05, 3.0, 2000);
zR_vec   = z_vec / R;

ratio_CB = 1 ./ (1 - (R ./ (4*z_vec)).^2);
ratio_CB(ratio_CB < 1 | ~isreal(ratio_CB)) = 1;

% Hayden 근사 (비교용)
ratio_HD = 1 + 0.16 * (R ./ z_vec).^2;

%% 추력 변화율 (수치 미분)
dTdz = abs(gradient(ratio_CB, z_vec));

%% 랜딩기어 높이 결정
% 기준 1 — 안전 기준: 프롭이 지면에 닿지 않는 최소 높이
h_safety = R - h_cg + 0.05;

% 기준 2 — 제어 교란 회피: z/R > 0.5 보장
%   z_min = 0.5*R + h_cg  이상이어야 함
%   → h_ctrl = 0.5*R
h_ctrl = 0.5 * R;

% 최종: 두 기준 중 큰 값
h_lg = max(h_safety, h_ctrl);
z_lg = h_lg + h_cg;       % 드론 중심 지상고 [m]
zR_lg = z_lg / R;          % 정규화 지상고 [-]

ratio_at_lg = 1 / (1 - (R/(4*z_lg))^2);

fprintf('\n[랜딩기어 높이 결정]\n');
fprintf('  기준1 h_safety = R - h_cg + 0.05 = %.0f mm\n', h_safety*1000);
fprintf('  기준2 h_ctrl   = 0.5 * R         = %.0f mm\n', h_ctrl*1000);
fprintf('  최종  h_lg     = %.0f mm\n', h_lg*1000);
fprintf('  → z_lg = %.3f m  (z/R = %.2f)\n', z_lg, zR_lg);
fprintf('  → T_IGE/T_OGE at h_lg = %.4f (+%.2f%%)\n', ...
    ratio_at_lg, (ratio_at_lg-1)*100);

%% 이륙 구간 추력 여유
% 랜딩기어 높이 → 임무 호버링 고도(1m) 구간
z_to   = linspace(z_lg, 1.0, 300);
r_to   = 1 ./ (1 - (R ./ (4*z_to)).^2);
r_to(r_to < 1) = 1;

T_per  = thrustHover_Est * 1e-3 * g_acc;          % 로터당 호버 추력 [N]
T_act  = T_per * RotorNo .* r_to;                  % 실제 총 추력 [N]
T_req  = mass_Total * 1e-3 * g_acc;                % MTOW [N]
margin = (T_act - T_req) ./ T_req * 100;           % 추력 여유 [%]

fprintf('\n[이륙 구간 추력 여유]\n');
fprintf('  MTOW = %.1f g\n', mass_Total);
fprintf('  이륙 직후 (z=%.3fm): +%.2f%%\n', z_to(1),  margin(1));
fprintf('  임무 고도 (z=1.000m): +%.2f%%\n', margin(end));

%% Fig 3: 지면효과 추력 비율
figure('Name','[3순위] 지면효과 추력 비율', 'Position',[50 50 800 500]);
plot(zR_vec, ratio_CB, 'b-',  'LineWidth',2.5); hold on;
plot(zR_vec, ratio_HD, 'r--', 'LineWidth',2.0);
xline(0.5,   'Color',[0 0.7 0], 'LineStyle','--', 'LineWidth',1.5, ...
    'Label','z/R=0.5 재순환 와류 경계', 'LabelVerticalAlignment','bottom');
xline(1.5,   'Color',[0.7 0 0.7], 'LineStyle','--', 'LineWidth',1.5, ...
    'Label','z/R=1.5 지면효과 소멸', 'LabelVerticalAlignment','bottom');
xline(zR_lg, 'k-', 'LineWidth',2.0, ...
    'Label', sprintf('★랜딩기어 z/R=%.2f', zR_lg), ...
    'LabelVerticalAlignment','bottom', 'LabelHorizontalAlignment','right');
yline(1.0, 'k:', 'LineWidth',1.0);
hold off; grid on;
xlim([0 3]); ylim([0.95 2.0]);
xlabel('정규화 지상고  z/R', 'FontSize',12);
ylabel('T_{IGE} / T_{OGE}', 'FontSize',12);
title('[3순위] 지면효과 추력 비율 — Cheeseman-Bennett / Hayden', 'FontSize',13);
legend({'Cheeseman-Bennett','Hayden 근사'}, 'Location','northeast','FontSize',11);

%% Fig 4: 추력 변화율 (제어 교란 지표)
dTdz_p = dTdz;
dTdz_p(zR_vec < 0.3) = NaN;   % 물리적 불가 구간 제거
dTdz_p(dTdz_p > 5.0) = NaN;   % 시각화 상한 클리핑

figure('Name','[3순위] 추력 변화율', 'Position',[50 50 800 500]);
plot(zR_vec, dTdz_p, 'b-', 'LineWidth',2.5); hold on;
xline(0.5,   'Color',[0 0.7 0], 'LineStyle','--', 'LineWidth',1.5, ...
    'Label','z/R=0.5 재순환 와류 경계');
xline(zR_lg, 'k-', 'LineWidth',2.0, ...
    'Label', sprintf('★랜딩기어 z/R=%.2f', zR_lg), ...
    'LabelHorizontalAlignment','right');
hold off; grid on;
xlim([0 3]); ylim([0 5.5]);
xlabel('정규화 지상고  z/R', 'FontSize',12);
ylabel('|dT_{ratio}/dz|  [1/m]', 'FontSize',12);
title('[3순위] 추력 변화율 — 제어 교란 최소화 설계 기준', 'FontSize',13);
annotation('textbox',[0.55 0.55 0.36 0.18], ...
    'String',{'z/R < 0.3 은 프롭이','지면에 닿는 영역 → 생략'}, ...
    'FitBoxToText','on','BackgroundColor',[1 1 0.8],'FontSize',9);

%% Fig 5: 이륙 구간 추력 여유
figure('Name','[3순위] 이륙 구간 추력 여유', 'Position',[50 50 800 500]);
yyaxis left
plot(z_to, r_to, 'b-', 'LineWidth',2.5);
ylabel('T_{IGE}/T_{OGE}', 'FontSize',12);

yyaxis right
plot(z_to, margin, 'r-', 'LineWidth',2.5);
yline(0,'k--','LineWidth',1.0);
ylabel('추력 여유  [%]', 'FontSize',12);

xlabel('드론 중심 지상고  z  [m]', 'FontSize',12);
title('[3순위] 이륙 구간 — 지면효과에 의한 추력 여유', 'FontSize',13);
grid on;

%% =========================================================================
% 최종 요약
% =========================================================================
fprintf('\n============= 공력 해석 요약 =============\n');
fprintf('[1순위] %s\n', prop_name);
fprintf('  DL     = %.4f g/cm²\n', DL(cp));
fprintf('  FM     = %.4f  (%.1f%%)\n', FM(cp), FM(cp)*100);
fprintf('  비추력 = %.4f g/W\n', ST(cp));
fprintf('[3순위] 랜딩기어\n');
fprintf('  h_lg   = %.0f mm  (z/R=%.2f)\n', h_lg*1000, zR_lg);
fprintf('  T_IGE/T_OGE = %.4f (+%.2f%%)\n', ratio_at_lg,(ratio_at_lg-1)*100);
fprintf('==========================================\n\n');