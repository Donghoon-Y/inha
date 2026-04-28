clc; clear; close all;

% 데이터 불러오기
aoa  = readmatrix('Homework I.xlsx', 'Sheet', 1, 'Range', 'A3:A62');
cg   = [0.2 0.25 0.3]; 
data = readmatrix('Homework I.xlsx', 'Sheet', 1, 'Range', 'A3:AH62');
data = data(:, ~any(isnan(data), 1));
dele = linspace(-9, 9, 7);

% 결과 변수
CL_range   = linspace(0, 1.3, 200);
trimcl     = repmat(CL_range', 1, 3);   % 모든 CG에 대해 같은 CL_range
trimcd     = zeros(length(CL_range), 3);
dele_trim  = zeros(length(CL_range), 3);
trimaoa    = zeros(length(CL_range), 3);

% === Cm 계산 (CG 보정) ===
for c = 1:3
    Cm = zeros(60, 7); 
    CL = zeros(60, 7); 
    CD = zeros(60, 7);
    for i = 1:7
        cm = 4*i; cl = 4*i - 2; cd = 4*i - 1;
        if c == 1 % CG=20%
            Cm(:,i) = data(:,cm) + data(:,cl)*(cg(1)-cg(2)).*cosd(aoa) ...
                                 - data(:,cd)*(cg(1)-cg(2)).*sind(aoa);
        elseif c == 2 % CG=25%
            Cm(:,i) = data(:,cm);
        else % CG=30%
            Cm(:,i) = data(:,cm) + data(:,cl)*(cg(3)-cg(2)).*cosd(aoa) ...
                                 - data(:,cd)*(cg(3)-cg(2)).*sind(aoa);
        end
        CL(:,i) = data(:,cl);
        CD(:,i) = data(:,cd);
    end

    % === CL 기준 보간 ===
    Cm_CL = zeros(length(CL_range), length(dele));
    CD_CL = zeros(length(CL_range), length(dele));
    AOA_CL = zeros(length(CL_range), length(dele));

    for j = 1:length(dele)
        Cm_CL(:,j)  = interp1(CL(:,j), Cm(:,j), CL_range);
        CD_CL(:,j)  = interp1(CL(:,j), CD(:,j), CL_range);
        AOA_CL(:,j) = interp1(CL(:,j), aoa,    CL_range);
    end

    % === Cm=0일 때 δe, CD, AOA 추출 ===
    for k = 1:length(CL_range)
        dele_trim(k,c) = interp1(Cm_CL(k,:), dele, 0);
        trimcd(k,c)    = interp1(dele, CD_CL(k,:),  dele_trim(k,c));
        trimaoa(k,c)   = interp1(dele, AOA_CL(k,:), dele_trim(k,c));
    end
end

W = 5000; % Weight [lb]
Sw = 200; % Area of winf [ft^2]
rho_SL = 0.002377; % Density at Sea Level [slug/ft^3]
rho_H = 0.001756; % Density at 10000 ft [slug/ft^3]

% Sea level 속도 계산
mac20_sl_vel = zeros(size(CL_range));
mac25_sl_vel = zeros(size(CL_range));
mac30_sl_vel = zeros(size(CL_range));	
mac20_10_vel = zeros(size(CL_range));
mac25_10_vel = zeros(size(CL_range));
mac30_10_vel = zeros(size(CL_range));	
for i = 1:length(CL_range)
    mac20_sl_vel(i) = sqrt((2*W)/(rho_SL*Sw*trimcl(i,1)));  
    mac25_sl_vel(i) = sqrt((2*W)/(rho_SL*Sw*trimcl(i,2)));
    mac30_sl_vel(i) = sqrt((2*W)/(rho_SL*Sw*trimcl(i,3)));
    mac20_10_vel(i) = sqrt((2*W)/(rho_H*Sw*trimcl(i,1)));  
    mac25_10_vel(i) = sqrt((2*W)/(rho_H*Sw*trimcl(i,2)));
    mac30_10_vel(i) = sqrt((2*W)/(rho_H*Sw*trimcl(i,3)));
end


mac20Preq_SL =zeros(length(trimcd),1);
mac25Preq_SL =zeros(length(trimcd),1);
mac30Preq_SL =zeros(length(trimcd),1);
mac20Preq_10 =zeros(length(trimcd),1);
mac25Preq_10 =zeros(length(trimcd),1);
mac30Preq_10 =zeros(length(trimcd),1);

mac20Treq_SL =zeros(length(trimcd),1);
mac25Treq_SL =zeros(length(trimcd),1);
mac30Treq_SL =zeros(length(trimcd),1);
mac20Treq_10 =zeros(length(trimcd),1);
mac25Treq_10 =zeros(length(trimcd),1);
mac30Treq_10 =zeros(length(trimcd),1);


for i = 1:length(mac20Preq_SL) 
    mac20Preq_SL(i) = 1/2*rho_SL*mac20_sl_vel(i)^3*Sw*trimcd(i,1)/550;
    mac25Preq_SL(i) = 1/2*rho_SL*mac25_sl_vel(i)^3*Sw*trimcd(i,2)/550;
    mac30Preq_SL(i) = 1/2*rho_SL*mac30_sl_vel(i)^3*Sw*trimcd(i,3)/550;
    mac20Preq_10(i) = 1/2*rho_H*mac20_10_vel(i)^3*Sw*trimcd(i,1)/550;
    mac25Preq_10(i) = 1/2*rho_H*mac25_10_vel(i)^3*Sw*trimcd(i,2)/550;
    mac30Preq_10(i) = 1/2*rho_H*mac30_10_vel(i)^3*Sw*trimcd(i,3)/550;

    mac20Treq_SL(i) = 0.5 * rho_SL * mac20_sl_vel(i)^2 * Sw * trimcd(i,1);
    mac25Treq_SL(i) = 0.5 * rho_SL * mac25_sl_vel(i)^2 * Sw * trimcd(i,2);
    mac30Treq_SL(i) = 0.5 * rho_SL * mac30_sl_vel(i)^2 * Sw * trimcd(i,3);
    mac20Treq_10(i) = 0.5 * rho_H * mac20_10_vel(i)^2 * Sw * trimcd(i,1);
    mac25Treq_10(i) = 0.5 * rho_H * mac25_10_vel(i)^2 * Sw * trimcd(i,2);
    mac30Treq_10(i) = 0.5 * rho_H * mac30_10_vel(i)^2 * Sw * trimcd(i,3);
end

D_min1 = min(mac20Treq_SL);  
V_plot = linspace(0, 500, 500);  
P_tangent1 = (D_min1 .* V_plot) / 550;  

D_min2 = min(mac25Treq_SL);   
P_tangent2 = (D_min2 .* V_plot) / 550;  

D_min3 = min(mac30Treq_SL);   
P_tangent3 = (D_min3 .* V_plot) / 550;  

figure;
hold on;
plot(mac20_sl_vel / 1.68781, mac20Preq_SL, LineWidth=1.25,DisplayName='Sea Level');
plot(mac20_10_vel / 1.68781, mac20Preq_10, LineWidth=1.25, DisplayName='10kft');
plot(V_plot/1.68781, P_tangent1, DisplayName='Treq Line', LineStyle='--', LineWidth=1.25);
title("CG : 20MAC, Power Required Curve");
xlabel("V(kts)");
ylabel("P(HP)");
grid on;
ylim([0,1200]);
xlim([0,300]);
legend();

hold off;

figure;
hold on;
plot(mac30_sl_vel / 1.68781, mac30Preq_SL, LineWidth=1.25,DisplayName='Sea Level');
plot(mac30_10_vel / 1.68781, mac30Preq_10, LineWidth=1.25, DisplayName='10kft');
plot(V_plot/1.68781, P_tangent3, DisplayName='Treq Line', LineStyle='--', LineWidth=1.25);
xlabel("V(kts)");
ylabel("P(HP)");
title("CG : 30MAC, Power Required Curve");
grid on;
ylim([0,1200]);
xlim([0,300]);
legend();

hold off;

figure;
hold on;
plot(mac25_sl_vel / 1.68781, mac25Preq_SL, LineWidth=1.25,DisplayName='Sea Level');
plot(mac25_10_vel / 1.68781, mac25Preq_10, LineWidth=1.25, DisplayName='10kft');
plot(V_plot/1.68781, P_tangent2, DisplayName='Treq Line', LineStyle='--', LineWidth=1.25);
xlabel("V(kts)");
ylabel("P(HP)");
title("CG : 25MAC, Power Required Curve");
grid on;
ylim([0,1200]);
xlim([0,300]);
legend();

hold off;


figure;
hold on;
plot(mac20_sl_vel / 1.68781, mac20Preq_SL, LineWidth=1.25,DisplayName='MAC 20');
plot(mac25_sl_vel / 1.68781, mac25Preq_SL, LineWidth=1.25,DisplayName='MAC 25');
plot(mac30_sl_vel / 1.68781, mac30Preq_SL, LineWidth=1.25,DisplayName='MAC 30');
xlabel("V(kts)");
ylabel("P(HP)");
title("Power Required Curve according to CG at Sea Level");
grid on;
ylim([0,1200]);
xlim([0,300]);
legend();

hold off;


figure;
hold on;
plot(mac20_10_vel / 1.68781, mac20Preq_10, LineWidth=1.25,DisplayName='MAC 20');
plot(mac25_10_vel / 1.68781, mac25Preq_10, LineWidth=1.25,DisplayName='MAC 25');
plot(mac30_10_vel / 1.68781, mac30Preq_10, LineWidth=1.25,DisplayName='MAC 30');
xlabel("V(kts)");
ylabel("P(HP)");
title("Power Required Curve according to CG at 10000ft");
grid on;
ylim([0,1200]);
xlim([0,300]);
legend();

hold off;


figure;
hold on;
plot(mac20_10_vel / 1.68781, mac20Preq_10, LineWidth=1.25,DisplayName='MAC 20 ,10kft');
plot(mac25_10_vel / 1.68781, mac25Preq_10, LineWidth=1.25,DisplayName='MAC 25, 10kft');
plot(mac30_10_vel / 1.68781, mac30Preq_10, LineWidth=1.25,DisplayName='MAC 30, 10kft');
plot(mac20_sl_vel / 1.68781, mac20Preq_SL, LineWidth=1.25,DisplayName='MAC 20, Sea Level');
plot(mac25_sl_vel / 1.68781, mac25Preq_SL, LineWidth=1.25,DisplayName='MAC 25, Sea Level');
plot(mac30_sl_vel / 1.68781, mac30Preq_SL, LineWidth=1.25,DisplayName='MAC 30, Sea Level');
xlabel("V(kts)");
ylabel("P(HP)");
title("Power Required Curve");
grid on;
ylim([0,1200]);
xlim([0,300]);
legend();

hold off;