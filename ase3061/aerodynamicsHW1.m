clc; clear all;

data = readmatrix("Homework I.xlsx"); %이 data 값은 0.25%c 지점을 기준으로 기술된 값이다.

%dele -9 data 저장
dele_m9_AOA = data(:, 1);
dele_m9_CL = data(:, 2);
dele_m9_CD = data(:, 3);
dele_m9_CM = data(:, 4);

%dele -6 data 저장
dele_m6_AOA = data(:, 6);
dele_m6_CL = data(:, 7);
dele_m6_CD = data(:, 8);
dele_m6_CM = data(:, 9);

%dele -3 data 저장
dele_m3_AOA = data(:, 11);
dele_m3_CL = data(:, 12);
dele_m3_CD = data(:, 13);
dele_m3_CM = data(:, 14);

%dele 0 data 저장
dele_0_AOA = data(:, 16);
dele_0_CL = data(:, 17);
dele_0_CD = data(:, 18);
dele_0_CM = data(:, 19);

%dele 3 data 저장
dele_3_AOA = data(:, 21);
dele_3_CL = data(:, 22);
dele_3_CD = data(:, 23);
dele_3_CM = data(:, 24);

%dele 6 data 저장
dele_6_AOA = data(:, 26);
dele_6_CL = data(:, 27);
dele_6_CD = data(:, 28);
dele_6_CM = data(:, 29);

%dele 9 data 저장
dele_9_AOA = data(:, 31);
dele_9_CL = data(:, 32);
dele_9_CD = data(:, 33);
dele_9_CM = data(:, 34);

%CL AOA를 이론값과 비교하기 위해서 2pi의 기울기를 가지는 직선을 만든다.
x = linspace(-3, 15, 100);
y = @(x) 2*pi*x*pi/180;
%AOA과 CL 비교 그래프
figure();
hold on;
plot(dele_m9_AOA, dele_m9_CL, 'DisplayName', 'dele = -9');
plot(dele_m6_AOA, dele_m6_CL, 'DisplayName', 'dele = -6');
plot(dele_m3_AOA, dele_m3_CL, 'DisplayName', 'dele = -3');
plot(dele_0_AOA, dele_0_CL, 'DisplayName', 'dele = 0');
plot(dele_3_AOA, dele_3_CL, 'DisplayName', 'dele = 3');
plot(dele_6_AOA, dele_6_CL, 'DisplayName', 'dele = 6');
plot(dele_9_AOA, dele_9_CL, 'DisplayName', 'dele = 9');
plot(x, y(x), LineStyle="--", DisplayName= "y=(2pi)*x")
xlabel('AOA(Angle of Attack');
ylabel('CL');
title("AOA vs CL");
legend();
grid(true);
hold off;

%CL과 CD 비교 그래프
figure();
hold on;
plot(dele_m9_CD, dele_m9_CL, "DisplayName", "dele = -9");
plot(dele_m6_CD, dele_m6_CL, "DisplayName", "dele = -6");
plot(dele_m3_CD, dele_m3_CL, "DisplayName", "dele = -3");
plot(dele_0_CD, dele_0_CL, "DisplayName", "dele = 0");
plot(dele_3_CD, dele_3_CL, "DisplayName", "dele = 3");
plot(dele_6_CD, dele_6_CL, "DisplayName", "dele = 6");
plot(dele_9_CD, dele_9_CL, "DisplayName", "dele = 9");
yline(0.6, color='red', LineWidth=2);
xlabel('CD');
ylabel('CL');
title("CD vs CL");
legend();
grid(true);
hold off;

% CL과 CM 비교 그래프(MRC : 25%MAC)
figure();
hold on;
%industry standard에 의해서 크기가 감소하는 방향의 축으로 설정하여 plot 한다.
plot(dele_m9_CM, dele_m9_CL, "DisplayName", "dele = -9");
plot(dele_m6_CM, dele_m6_CL, "DisplayName", "dele = -6");
plot(dele_m3_CM, dele_m3_CL, "DisplayName", "dele = -3");
plot(dele_0_CM, dele_0_CL, "DisplayName", "dele = 0");
plot(dele_3_CM, dele_3_CL, "DisplayName", "dele = 3");
plot(dele_6_CM, dele_6_CL, "DisplayName", "dele = 6");
plot(dele_9_CM, dele_9_CL, "DisplayName", "dele = 9");
yline(0.6, color='red', LineWidth=2);
set(gca, 'XDir','reverse');
xlabel('CM');
ylabel('CL');
title("CM vs CL, MRC : 25% MAC");
legend();
grid(true);
hold off;

%%



