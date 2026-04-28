clc; clear all;

%%%% 데이터 불러오기 %%%%
data_15bar_1 = importdata('RUN003.lvm');
data_15bar_2 = importdata('RUN004.lvm');
data_20bar_1 = importdata('RUN005.lvm');
data_20bar_2 = importdata('RUN006.lvm');

%%%model Parameter%%%
P0 = 1.013;   % Atmospheric pressure [bar]
gamma = 1.4;
T0 = 300;
R = 297;
Me = 2.94;
At = 0.002^2*pi;
Ae = 4*At ;
Te = T0*((1+((gamma-1)/2)*Me^2)^(-1));
a = sqrt(gamma*R*Te);
Ve = a*Me;

%%%%%%%%%%%%%%% 15bar Test1 %%%%%%%%%%%%%%%
test1_t_15bar = data_15bar_1(:,1);
test1_P_15bar_raw   = data_15bar_1(:,3); 
test1_Load_15bar_raw = data_15bar_1(:,2);

% 1) 이동평균 필터
test1_Load_15bar = smoothdata(test1_Load_15bar_raw, "movmean", 50);
test1_P_15bar    = smoothdata(test1_P_15bar_raw,   "movmean", 50);

% 2) baseline offset
thrust_base_15_1   = mean(test1_Load_15bar(1:10000));
pressure_base_15_1 = mean(test1_P_15bar(1:10000));

% 3) 추력 계산
test1_Thrust_15bar = (test1_Load_15bar - thrust_base_15_1) * 7.711;

% 4) 압력 보정 (bar 그대로)
test1_P_15bar_bar = (test1_P_15bar - pressure_base_15_1)*10 ;



%%%%%%%%%%%%%%% 15bar Test2 %%%%%%%%%%%%%%%
test2_t_15bar = data_15bar_2(:,1);
test2_P_15bar_raw   = data_15bar_2(:,3);
test2_Load_15bar_raw = data_15bar_2(:,2);

test2_Load_15bar = smoothdata(test2_Load_15bar_raw, "movmean", 50);
test2_P_15bar    = smoothdata(test2_P_15bar_raw,   "movmean", 50);

thrust_base_15_2   = mean(test2_Load_15bar(1:10000));
pressure_base_15_2 = mean(test2_P_15bar(1:10000));

test2_Thrust_15bar = (test2_Load_15bar - thrust_base_15_2) * 7.711;

test2_P_15bar_bar = (test2_P_15bar - pressure_base_15_2)*10 ;



%%%%%%%%%%%%%%% 20bar Test1 %%%%%%%%%%%%%%%
test1_t_20bar = data_20bar_1(:,1);
test1_P_20bar_raw   = data_20bar_1(:,3);
test1_Load_20bar_raw = data_20bar_1(:,2);

test1_Load_20bar = smoothdata(test1_Load_20bar_raw, "movmean", 50);
test1_P_20bar    = smoothdata(test1_P_20bar_raw,   "movmean", 50);

thrust_base_20_1   = mean(test1_Load_20bar(1:10000));
pressure_base_20_1 = mean(test1_P_20bar(1:10000));

test1_Thrust_20bar = (test1_Load_20bar - thrust_base_20_1) * 7.711;

test1_P_20bar_bar = (test1_P_20bar - pressure_base_20_1)*10;



%%%%%%%%%%%%%%% 20bar Test2 %%%%%%%%%%%%%%%
test2_t_20bar = data_20bar_2(:,1);
test2_P_20bar_raw   = data_20bar_2(:,3);
test2_Load_20bar_raw = data_20bar_2(:,2);

test2_Load_20bar = smoothdata(test2_Load_20bar_raw, "movmean", 50);
test2_P_20bar    = smoothdata(test2_P_20bar_raw,   "movmean", 50);

thrust_base_20_2   = mean(test2_Load_20bar(1:10000));
pressure_base_20_2 = mean(test2_P_20bar(1:10000));

test2_Thrust_20bar = (test2_Load_20bar - thrust_base_20_2) * 7.711;
test2_P_20bar_bar = (test2_P_20bar - pressure_base_20_2)*10 ;


%%%%평균 추력 출력%%%%
figure();
hold on;
plot(test1_t_15bar, test1_Thrust_15bar, DisplayName="Test1-15bar");
plot(test2_t_15bar, test2_Thrust_15bar, DisplayName="Test2-15bar");
plot(test1_t_20bar, test1_Thrust_20bar, DisplayName="Test1-20bar");
plot(test2_t_20bar, test2_Thrust_20bar, DisplayName="Test2-20bar");
legend()
hold off;

mean_Thrust_test1_15bar = mean(test1_Thrust_15bar(42000:58000));
mean_Thrust_test2_15bar = mean(test2_Thrust_15bar(41000:56000));
mean_Thrust_test1_20bar = mean(test1_Thrust_20bar(54000:63000));
mean_Thrust_test2_20bar = mean(test2_Thrust_20bar(48000:57000));

disp("mean Thrust Results");
disp(mean_Thrust_test1_15bar);
disp(mean_Thrust_test2_15bar);
disp(mean_Thrust_test1_20bar);
disp(mean_Thrust_test2_20bar);

%%%%평균 압력 및 질유량 계산
figure();
hold on;
plot(test1_t_15bar, test1_P_15bar_bar, DisplayName="Test1-15bar");
plot(test2_t_15bar, test2_P_15bar_bar, DisplayName="Test2-15bar");
plot(test1_t_20bar, test1_P_20bar_bar, DisplayName="Test1-20bar");
plot(test2_t_20bar, test2_P_20bar_bar, DisplayName="Test2-20bar");
legend();
hold off;

mean_P_test1_15bar = mean(test1_P_15bar_bar(21000:57000));
mean_P_test2_15bar = mean(test2_P_15bar_bar(25000:54000));
mean_P_test1_20bar = mean(test1_P_20bar_bar(26000:53000));
mean_P_test2_20bar = mean(test2_P_20bar_bar(30000:60000));

m_dot_15_1 = At*(mean_P_test1_15bar)*(10^5)*sqrt(2/(gamma+1))^((gamma+1)/(gamma-1))*sqrt(gamma/R/T0);
m_dot_15_2 = At*(mean_P_test2_15bar)*(10^5)*sqrt(2/(gamma+1))^((gamma+1)/(gamma-1))*sqrt(gamma/R/T0);
m_dot_20_1 = At*(mean_P_test1_20bar)*(10^5)*sqrt(2/(gamma+1))^((gamma+1)/(gamma-1))*sqrt(gamma/R/T0);
m_dot_20_2 = At*(mean_P_test2_20bar)*(10^5)*sqrt(2/(gamma+1))^((gamma+1)/(gamma-1))*sqrt(gamma/R/T0);

disp("mean mass flow results");
disp(m_dot_15_1);
disp(m_dot_15_2);
disp(m_dot_20_1);
disp(m_dot_20_2); 

%%%%출구 압력 계산%%%%
Pe_15_1 = (mean_P_test1_15bar)*(1+((gamma-1)/2)*Me^2)^(-gamma/(gamma-1));
Pe_15_2 = (mean_P_test2_15bar)*(1+(gamma-1)/2*Me^2)^(-gamma/(gamma-1));
Pe_20_1 = (mean_P_test1_20bar)*(1+(gamma-1)/2*Me^2)^(-gamma/(gamma-1));
Pe_20_2 = (mean_P_test2_20bar)*(1+(gamma-1)/2*Me^2)^(-gamma/(gamma-1));

disp("mean Exit Pressure");
disp(Pe_15_1);
disp(Pe_15_2);
disp(Pe_20_1);
disp(Pe_20_2);


%%%%%2.2 code%%%%%

%%%%추력 계산 %%%%
cal_Thrust_15bar_test1 = m_dot_15_1*Ve + Ae*((Pe_15_1)*10^5);
cal_Thrust_15bar_test2 = m_dot_15_2*Ve + Ae*(Pe_15_2)*10^5;
cal_Thrust_20bar_test1 = m_dot_20_1*Ve + Ae*(Pe_20_1)*10^5;
cal_Thrust_20bar_test2 = m_dot_20_2*Ve + Ae*(Pe_20_2)*10^5;

disp("cal Thrust");
disp(cal_Thrust_15bar_test1);
disp(cal_Thrust_15bar_test2);
disp(cal_Thrust_20bar_test1);
disp(cal_Thrust_20bar_test2);

%%%상대오차 계산
error_15bar_test1 = abs(cal_Thrust_15bar_test1-mean_Thrust_test1_15bar)/cal_Thrust_15bar_test1*100;
error_15bar_test2 = abs(cal_Thrust_15bar_test2-mean_Thrust_test2_15bar)/cal_Thrust_15bar_test2*100;
error_20bar_test1 = abs(cal_Thrust_20bar_test1-mean_Thrust_test1_20bar)/cal_Thrust_20bar_test1*100;
error_20bar_test2 = abs(cal_Thrust_20bar_test2-mean_Thrust_test2_20bar)/cal_Thrust_20bar_test2*100;

disp("Relative Error");
disp(error_15bar_test1);
disp(error_15bar_test2);
disp(error_20bar_test1);
disp(error_20bar_test2);

%% 2.3 시간에 따른 챔버의 정체압력 계산 (4가지 실험 데이터)

disp("=========== 2.3: ODE 기반 챔버 압력 계산 시작 ===========");

%% 문제 조건
Tin = 300;     
gamma = 1.4;
R = 297;

% 챔버 형상
d_chamber = 9e-3;
L_chamber = 0.14;
Volume_c = pi*(d_chamber/2)^2 * L_chamber;

% 노즐 throat
d_nozzle = 4e-3;
A_t = pi*(d_nozzle/2)^2;

% 초기 챔버 상태 (1 bar)
Pc_init = 1e5;
tspan = [0 0.5];

%% 실험에서 얻은 평균 압력(bar)
target_pressure_list = [
    mean_P_test1_15bar;
    mean_P_test2_15bar;
    mean_P_test1_20bar;
    mean_P_test2_20bar
];

case_names = {
    "15bar Test1", "15bar Test2", ...
    "20bar Test1", "20bar Test2"
};

%% 결과 저장
m_in_results = zeros(4,1);
Pc_curve_list = cell(4,1);   % ← ★★★ ODE 결과 저장 리스트 추가 ★★★
t_curve_list = cell(4,1);

%% 4개 실험 케이스 수행
for k = 1:4
    
    target_P_bar = target_pressure_list(k);
    name = case_names{k};

    fprintf("\n### %s: 목표 Pc = %.3f bar ###\n", name, target_P_bar);

    % ---- 함수 호출 ----
    [m_in_est, t_curve, Pc_curve] = find_m_in( ...
        target_P_bar, Tin, gamma, R, ...
        Volume_c, A_t, Pc_init, tspan);

    m_in_results(k) = m_in_est;

    % ---- 계산 결과 저장 ----
    Pc_curve_list{k} = Pc_curve;     % ★★★★★ 저장 ★★★★★
    t_curve_list{k} = t_curve;

    % ---- 개별 그래프 ----
    figure;
    plot(t_curve, Pc_curve/1e5, 'LineWidth', 1);
    xlabel("time [s]");
    ylabel("Pc [bar]");
    title(sprintf("Chamber Pressure vs Time (%s)", name));
    grid on;

    fprintf("  → m_in ≈ %.5f kg/s\n", m_in_est);
end

%% ===================== Sensor P vs Total P 출력 =====================

fprintf("\n================ Sensor P vs Total P Comparison ================\n");

for k = 1:4
    sensorP = target_pressure_list(k);           % 실험 압력
    totalP  = Pc_curve_list{k}(end) / 1e5;       % ODE 결과 압력
    err     = abs(totalP - sensorP) / sensorP * 100;

    fprintf("Case %-12s | Sensor P = %.4f bar | Total P = %.4f bar | Err = %.3f %%\n", ...
        case_names{k}, sensorP, totalP, err);
end

fprintf("=================================================================\n");


%% 2.4 Pin – uin 그래프 및 Table 5 + u=200 m/s에서 Pin 계산


disp("=========== 2.4: Pin – uin 그래프 및 Table 5 계산 시작 ===========");

% ---- 기본 상수 ----
Tin = 300;
gamma = 1.4;
R = 297;

% Inlet 형상 (문제 그림 기준 d = 4.2 mm)
d_inlet = 4.2e-3;              % [m]
A_in = pi*(d_inlet/2)^2;       % [m^2]

% 속도 범위 (y축)
u_range = linspace(100, 1000, 300);   % [m/s]

% 우리가 따로 보고 싶은 속도
u_target = 200;                        % [m/s]

% 결과 저장
Pinuin_list   = zeros(4,1);   % Pin * uin (kg/s^3)
Pin200_list   = zeros(4,1);   % u=200 m/s에서 Pin (bar)
massflow_gps  = m_in_results * 1000;   % kg/s -> g/s

figure; hold on;
colors = {'b','r','g','k'};

for k = 1:4
    m_in = m_in_results(k);   % 2.3에서 구한 m_in [kg/s]

    % --- Pin(u) 관계식 ---
    Pin_bar = (m_in * R * Tin) ./ (u_range * A_in) / 1e5;   % [bar]

    % u=200 m/s에서 필요한 Pin
    Pin_200_bar = (m_in * R * Tin) / (u_target * A_in) / 1e5;
    Pin200_list(k) = Pin_200_bar;

    % Pin * uin = const (식 13)
    Pinuin_list(k) = Pin_200_bar * u_target * 1e5;

    % ---- 그래프 (x: Pin, y: u) ----
    plot(Pin_bar, u_range, 'Color', colors{k}, ...
         'LineWidth', 1.5, 'DisplayName', case_names{k});
end

% ======= 여기서 수평선만 추가 =======
yline(u_target, '--k', 'LineWidth', 1.2);

xlabel("P_{in} [bar(abs)]");
ylabel("u_{in} [m/s]");
title("P_{in} – u_{in} 관계 (u_{in}=200 m/s 수평선 추가)");
grid on;
legend("Location","northeast");

% --------- Table 5 형식으로 값 출력 ---------
disp("============= Table 5: Calculated Mass flow & Pin·uin =============");
fprintf("Case\t\t m_dot_total [g/s]\t P_in u_in [kg/s^3]\t P_in(u=200m/s) [bar]\n");
for k = 1:4
    fprintf("%-10s\t %8.4f\t\t %11.3e\t\t %7.3f\n", ...
        case_names{k}, massflow_gps(k), Pinuin_list(k), Pin200_list(k));
end
disp("====================================================================");

%% 함수: target Pc(bar)를 만족하는 m_in 추정 + 압력 곡선 반환
function [m_in_est, t_out, Pc_out] = find_m_in(target_P_bar, ...
                                               Tin, gamma, R, ...
                                               Volume_c, A_t, ...
                                               Pc_init, tspan)

    target_P = target_P_bar * 1e5;   % bar → Pa 변환

    % --- 초킹 유량식 ---
    m_dot_out = @(P) A_t * P * sqrt(gamma/(R*Tin)) * ...
                     (2/(gamma+1))^((gamma+1)/(2*(gamma-1)));

    % --- ODE ---
    odefun = @(t, Pc, m_in) (m_in - m_dot_out(Pc)) * R * Tin / Volume_c;

    % --- m_in 스캔 범위 (0.1 g/s ~ 20 g/s) ---
    found = false;
    for m_in_test = linspace(1e-4, 0.02, 400)   % kg/s
        
        [~, Pc] = ode45(@(t, Pc) odefun(t, Pc, m_in_test), tspan, Pc_init);
        
        if abs(Pc(end) - target_P) < 1e3      % ±0.01 bar 이내 도달
            m_in_est = m_in_test;
            found = true;
            break;
        end
    end

    if ~found
        warning("m_in을 찾지 못했습니다. 스캔 범위를 조정하세요.");
        m_in_est = NaN;
        t_out = [];
        Pc_out = [];
        return;
    end

    % 최종 해 다시 계산
    [t_out, Pc_out] = ode45(@(t, Pc) odefun(t, Pc, m_in_est), tspan, Pc_init);
end
