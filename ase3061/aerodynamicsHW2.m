clc; clear all;

data = readmatrix("Homework I.xlsx"); %이 data 값은 0.25%c 지점을 기준으로 기술된 값이다.

%dele -9 data 저장
dele_m9_CL = data(:, 2);
dele_m9_CM = data(:, 4);

%dele -6 data 저장
dele_m6_CL = data(:, 7);
dele_m6_CM = data(:, 9);

%dele -3 data 저장
dele_m3_CL = data(:, 12);
dele_m3_CM = data(:, 14);

%dele 0 data 저장
dele_0_CL = data(:, 17);
dele_0_CM = data(:, 19);

%dele 3 data 저장
dele_3_CL = data(:, 22);
dele_3_CM = data(:, 24);

%dele 6 data 저장
dele_6_CL = data(:, 27);
dele_6_CM = data(:, 29);

%dele 9 data 저장
dele_9_CL = data(:, 32);
dele_9_CM = data(:, 34);

% MRC : 20%MAC
dele_m9_CM_20 = dele_m9_CM + dele_m9_CL.*(1/5 - 1/4);
dele_m6_CM_20 = dele_m6_CM + dele_m6_CL.*(1/5 - 1/4);
dele_m3_CM_20 = dele_m3_CM + dele_m3_CL.*(1/5 - 1/4);
dele_0_CM_20 = dele_0_CM + dele_0_CL.*(1/5 - 1/4);
dele_3_CM_20 = dele_3_CM + dele_3_CL.*(1/5 - 1/4);
dele_6_CM_20 = dele_6_CM + dele_6_CL.*(1/5 - 1/4);
dele_9_CM_20 = dele_9_CM + dele_9_CL.*(1/5 - 1/4);

% MRC : 30%MAC
dele_m9_CM_30 = dele_m9_CM + dele_m9_CL.*(0.3 - 1/4);
dele_m6_CM_30 = dele_m6_CM + dele_m6_CL.*(0.3 - 1/4);
dele_m3_CM_30 = dele_m3_CM + dele_m3_CL.*(0.3 - 1/4);
dele_0_CM_30 = dele_0_CM + dele_0_CL.*(0.3 - 1/4);
dele_3_CM_30 = dele_3_CM + dele_3_CL.*(0.3 - 1/4);
dele_6_CM_30 = dele_6_CM + dele_6_CL.*(0.3 - 1/4);
dele_9_CM_30 = dele_9_CM + dele_9_CL.*(0.3 - 1/4);

figure();
hold on;
%industry standard에 의해서 크기가 감소하는 방향의 축으로 설정하여 plot 한다.
plot(dele_m9_CM_20, dele_m9_CL, "DisplayName", "dele = -9");
plot(dele_m6_CM_20, dele_m6_CL, "DisplayName", "dele = -6");
plot(dele_m3_CM_20, dele_m3_CL, "DisplayName", "dele = -3");
plot(dele_0_CM_20, dele_0_CL, "DisplayName", "dele = 0");
plot(dele_3_CM_20, dele_3_CL, "DisplayName", "dele = 3");
plot(dele_6_CM_20, dele_6_CL, "DisplayName", "dele = 6");
plot(dele_9_CM_20, dele_9_CL, "DisplayName", "dele = 9");
set(gca, 'XDir','reverse');
xlabel('CM');
ylabel('CL');
title("CM vs CL, MRC : 20% MAC");
legend();
grid();
hold off;

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
set(gca, 'XDir','reverse');
xlabel('CM');
ylabel('CL');
title("CM vs CL, MRC : 25% MAC");
legend();
grid();
hold off;

figure();
hold on;
%industry standard에 의해서 크기가 감소하는 방향의 축으로 설정하여 plot 한다.
plot(dele_m9_CM_30, dele_m9_CL, "DisplayName", "dele = -9");
plot(dele_m6_CM_30, dele_m6_CL, "DisplayName", "dele = -6");
plot(dele_m3_CM_30, dele_m3_CL, "DisplayName", "dele = -3");
plot(dele_0_CM_30, dele_0_CL, "DisplayName", "dele = 0");
plot(dele_3_CM_30, dele_3_CL, "DisplayName", "dele = 3");
plot(dele_6_CM_30, dele_6_CL, "DisplayName", "dele = 6");
plot(dele_9_CM_30, dele_9_CL, "DisplayName", "dele = 9");
set(gca, 'XDir','reverse');
xlabel('CM');
ylabel('CL');
title("CM vs CL, MRC : 30% MAC");
legend();
grid on;
hold off;