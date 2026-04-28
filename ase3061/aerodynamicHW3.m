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

% ===== Plotting =====

% (1) CL vs CD
figure(1); 
hold on; 
grid on;
plot(trimcd, trimcl, 'LineWidth', 1.5)
title('C_{D} vs C_{L}')
xlabel('C_{D}'); ylabel('C_{L}')
legend('CG : 20%MAC', 'CG : 25%MAC', 'CG : 30%MAC')
hold off

% (2) Elevator vs CL
figure(2); 
hold on; 
grid on;
plot(dele_trim, trimcl, 'LineWidth', 1.5)
xlabel('Elevator deflection (deg)')
ylabel('C_{L}')
legend('CG : 20%MAC', 'CG : 25%MAC', 'CG : 30%MAC')
title('Elevator vs C_{L}')
hold off

% (3) AOA vs CL
figure(3); 
hold on; 
grid on;
plot(trimaoa, trimcl, 'LineWidth', 1.5)
xlabel('AOA (deg)')
ylabel('C_{L}')
legend('CG : 20%MAC', 'CG : 25%MAC', 'CG : 30%MAC')
title('AOA vs C_{L}')
hold off
