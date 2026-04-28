%% ==========================================================
%  PERFORMANCE ANALYSIS (CG = 25% MAC)
%  5000 lb, 500 HP turboprop, η = 0.85
% ==========================================================
clc; clear; close all;

%% === 공력 데이터 ===
aoa  = readmatrix('Homework I.xlsx', 'Sheet', 1, 'Range', 'A3:A62');
data = readmatrix('Homework I.xlsx', 'Sheet', 1, 'Range', 'A3:AH62');
data = data(:, ~any(isnan(data), 1));
dele = linspace(-9, 9, 7);
cg   = [0.2 0.25 0.3];

%% === CG = 25%에서의 원데이터(CL, CD, Cm) 추출 ===
CL_m = zeros(60,7); CD_m = zeros(60,7); CM_m = zeros(60,7);
for i = 1:7
    cl = 4*i - 2; cd = 4*i - 1; cm = 4*i;
    CL_m(:,i) = data(:,cl);
    CD_m(:,i) = data(:,cd);
    CM_m(:,i) = data(:,cm);
end

%% === CL 범위 (0 방지 + 데이터 기반) ===
CL_min_raw  = min(CL_m(:));
CL_max_data = max(CL_m(:));
CL_min_data = max(CL_min_raw, 1e-4); % Inf 방지
nCL = 200;
CL_range = linspace(CL_min_data, CL_max_data, nCL)';

%% === Cm=0에서의 CD, AoA 보간 ===
trimcl = CL_range; trimcd = zeros(nCL,1); trimaoa = zeros(nCL,1);
for k = 1:nCL
    CM_at_CL = zeros(1,7); CD_at_CL = zeros(1,7); AOA_at_CL= zeros(1,7);
    for j = 1:7
        [CL_sorted, idx] = sort(CL_m(:,j));
        CD_sorted  = CD_m(idx,j);
        CM_sorted  = CM_m(idx,j);
        AOA_sorted = aoa(idx,1);
        CM_at_CL(j)  = interp1(CL_sorted, CM_sorted,  CL_range(k), 'linear', 'extrap');
        CD_at_CL(j)  = interp1(CL_sorted, CD_sorted,  CL_range(k), 'linear', 'extrap');
        AOA_at_CL(j) = interp1(CL_sorted, AOA_sorted, CL_range(k), 'linear', 'extrap');
    end
    dele_star = interp1(CM_at_CL, dele, 0, 'linear', 'extrap');
    trimcd(k) = interp1(dele, CD_at_CL,  dele_star, 'linear', 'extrap');
    trimaoa(k)= interp1(dele, AOA_at_CL, dele_star, 'linear', 'extrap');
end

%% === 기본 파라미터 ===
W=5000; S=200;
rho_SL=0.002377; rho_10=0.001756;
eta=0.85; HP_SL=500; HP_to_ftlb_s=550;
g=32.174; ft2kt = 1/1.687;

%% === Power Required & Available ===
V_SL = sqrt(2*W./(rho_SL*S*trimcl));
V_10 = sqrt(2*W./(rho_10*S*trimcl));
P_req_SL = 0.5*rho_SL.*V_SL.^3*S.*trimcd/550;
P_req_10 = 0.5*rho_10.*V_10.^3*S.*trimcd/550;
P_av_SL  = HP_SL*eta;
P_av_10  = HP_SL*eta*(rho_10/rho_SL);

%% === 최대속도 (fzero + 유한값 필터링) ===
maskSL = isfinite(V_SL) & isfinite(P_req_SL);
mask10 = isfinite(V_10) & isfinite(P_req_10);
VSLf = V_SL(maskSL); PRSLf = P_req_SL(maskSL);
V10f = V_10(mask10); PR10f = P_req_10(mask10);
f_SL = @(V) interp1(VSLf, PRSLf, V, 'linear', 'extrap') - P_av_SL;
f_10 = @(V) interp1(V10f, PR10f, V, 'linear', 'extrap') - P_av_10;
[~, idx_SL_approx] = min(abs(PRSLf - P_av_SL));
[~, idx_10_approx] = min(abs(PR10f - P_av_10));
Vmax_SL = fzero(f_SL, [VSLf(max(idx_SL_approx-1,1)), VSLf(min(idx_SL_approx+1,end))]);
Vmax_10 = fzero(f_10, [V10f(max(idx_10_approx-1,1)), V10f(min(idx_10_approx+1,end))]);
Pmax_SL = interp1(VSLf, PRSLf, Vmax_SL);
Pmax_10 = interp1(V10f, PR10f, Vmax_10);

%% === 실속속도 ===
Vs_SL = sqrt(2*W/(rho_SL*S*CL_max_data));
Vs_10 = sqrt(2*W/(rho_10*S*CL_max_data));

%% === Power Curve ===
figure(1)
plot(V_SL*ft2kt,P_req_SL,'r','LineWidth',1.5); hold on;
plot(V_10*ft2kt,P_req_10,'b','LineWidth',1.5);
yline(P_av_SL,'k--','LineWidth',1.2);
yline(P_av_10,'k:','LineWidth',1.2);
plot(Vmax_SL*ft2kt,Pmax_SL,'ko','MarkerFaceColor','g');
plot(Vmax_10*ft2kt,Pmax_10,'ko','MarkerFaceColor','y');
xlim([0,300]); ylim([0,600]);
xlabel('Velocity [kt]'); ylabel('Power [HP]');
title('Power Required vs Available (CG = 25%)');
legend('SL Required','10,000 ft Required','SL Available','10,000 ft Available','Location','northwest');
grid on;
text(Vmax_SL*ft2kt+5,Pmax_SL, sprintf('V_{max,SL}=%.1f kt',Vmax_SL*ft2kt), 'Color','k');
text(Vmax_10*ft2kt+5,Pmax_10, sprintf('V_{max,10k}=%.1f kt',Vmax_10*ft2kt), 'Color','k');

%% === Rate of Climb ===
ROC_SL = ((P_av_SL - P_req_SL)*HP_to_ftlb_s/W)*60;
ROC_10 = ((P_av_10 - P_req_10)*HP_to_ftlb_s/W)*60;
[ROCmax_SL,idx_rocSL] = max(ROC_SL);
[ROCmax_10,idx_roc10] = max(ROC_10);
figure(2)
plot(V_SL*ft2kt,ROC_SL,'r','LineWidth',1.5); hold on;
plot(V_10*ft2kt,ROC_10,'b','LineWidth',1.5);
plot(V_SL(idx_rocSL)*ft2kt,ROCmax_SL,'ko','MarkerFaceColor','g','MarkerSize',7);
plot(V_10(idx_roc10)*ft2kt,ROCmax_10,'ko','MarkerFaceColor','y','MarkerSize',7);
text(V_SL(idx_rocSL)*ft2kt+5,ROCmax_SL, sprintf('%.0f fpm @ %.1f kt',ROCmax_SL,V_SL(idx_rocSL)*ft2kt), 'Color','k');
text(V_10(idx_roc10)*ft2kt+5,ROCmax_10, sprintf('%.0f fpm @ %.1f kt',ROCmax_10,V_10(idx_roc10)*ft2kt), 'Color','k');
xlabel('True Airspeed [kt]'); ylabel('Rate of Climb [fpm]');
xlim([0,300]); ylim([0,3000]);
title('Rate of Climb (CG = 25%MAC)');
legend('Sea Level','10,000 ft','Max ROC (SL)','Max ROC (10kft)','Location','northwest');
grid on;

%% === Service Ceiling ===
alts=(0:1000:40000)'; ROCmax_alt=zeros(size(alts));
for j=1:numel(alts)
    rho_j = rho_SL*(1-6.875e-6*alts(j)).^4.256;
    P_avj = HP_SL*eta*(rho_j/rho_SL);
    Vj = sqrt(2*W./(rho_j*S*trimcl));
    PRj = 0.5*rho_j.*Vj.^3.*S.*trimcd/550;
    ROCj = ((P_avj-PRj)*HP_to_ftlb_s/W)*60;
    ROCmax_alt(j) = max(ROCj);
end
k_hi=find(ROCmax_alt>=100,1,'last'); k_lo=k_hi+1;
if ~isempty(k_hi) && k_lo<=numel(ROCmax_alt)
    svcCeil = interp1(ROCmax_alt([k_hi k_lo]), alts([k_hi k_lo]), 100);
else
    svcCeil = NaN;
end
figure(3)
plot(ROCmax_alt,alts,'LineWidth',1.5); hold on;
xline(100,'m--');
if ~isnan(svcCeil)
    yline(svcCeil,'k:');
    plot(100,svcCeil,'ko','MarkerFaceColor','g','MarkerSize',7);
    text(110,svcCeil+500,sprintf('%.0f ft',svcCeil),'Color','k','FontWeight','bold');
end
xlabel('Rate of Climb [fpm]'); ylabel('Altitude [ft]');
title('Service Ceiling (CG = 25%)');
legend('ROC_{max}(altitude)','100 fpm line','Service Ceiling','Location','southeast');
grid on;

%% === Sustained Turn Rate (수정 버전) ===
rho = rho_SL;
P_avail = HP_SL * eta * HP_to_ftlb_s;   % [ft·lb/s] 단위
mask = isfinite(trimcl) & isfinite(trimcd) & trimcl > 0;

CLg = trimcl(mask);
CDg = trimcd(mask);

% === 항력계수 식에서 CD0, K 추정 ===
X = [ones(length(CLg),1), CLg.^2];
theta = X \ CDg;  
CD0 = theta(1);
K = theta(2);

% === 속도 스캔 (실속 이상 ~ Vmax)
Vscan = linspace(Vs_SL*1.03, Vmax_SL, 600)';  % [ft/s]
qscan = 0.5 * rho .* (Vscan.^2);              % 동압 [lbf/ft^2]

% === 하중계수 한계 계산
num = (P_avail - Vscan .* (qscan * S * CD0)) .* (qscan * S);
den = Vscan .* K * (W^2);
n_power = sqrt(max(num ./ den, 0));
n_CL = (CL_max_data .* qscan * S) / W;
n_allow = min(n_power, n_CL);

% === 선회율 계산 ===
TRscan = (g ./ Vscan) .* sqrt(max(n_allow.^2 - 1, 0)) * 180/pi;  % [deg/s]

% === 최대 지속 선회율 추출 ===
[TR_max, idx_max] = max(TRscan);
n_star = n_allow(idx_max);
V_star = Vscan(idx_max);

% === 그래프 ===
figure(4);
plot(Vscan*ft2kt, TRscan, 'k', 'LineWidth', 1.6); hold on; grid on;
plot(V_star*ft2kt, TR_max, 'ro', 'MarkerFaceColor', 'y');
xlabel('Velocity [kt]');
ylabel('Turn Rate [deg/s]');
title('Maximum Sustained Turn Rate (Sea Level, 25%MAC)');
text(V_star*ft2kt + 3, TR_max + 0.5, ...
     sprintf('%.2f°/s @ n=%.2f, V=%.1f kt', TR_max, n_star, V_star*ft2kt), ...
     'Color', 'k', 'FontWeight', 'bold');
legend('Sustained Turn Rate', 'Max Point', 'Location', 'best');

% === 결과 출력 ===
fprintf('\n==============================\n');
fprintf(' PERFORMANCE SUMMARY (CG=25%%MAC)\n');
fprintf('==============================\n');
fprintf('CL_max (data)       : %.3f\n', CL_max_data);
fprintf('Vstall (SL)         : %.2f kt\n', Vs_SL*ft2kt);
fprintf('Vmax (SL,10kft)     : %.1f kt, %.1f kt\n', Vmax_SL*ft2kt, Vmax_10*ft2kt);
fprintf('Max ROC (SL,10kft)  : %.0f fpm, %.0f fpm\n', ROCmax_SL, ROCmax_10);
fprintf('Service Ceiling     : %.0f ft\n', svcCeil);
fprintf('Max Sustained TR(SL): %.2f deg/s @ n=%.2f, V=%.1f kt\n', TR_max, n_star, V_star*ft2kt);
fprintf('==============================\n');
