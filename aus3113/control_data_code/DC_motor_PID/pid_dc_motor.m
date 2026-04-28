clc; clear all;

data_fail = readmatrix('pid_log_fail.csv');

data_nonfail = readmatrix("pid_log_nonfail.csv");

t_fail = (1:length(data_fail));
wref_data_fail = data_fail(1:end,2);
w_meas_data_fail = data_fail(1:end,3);

t_nonfail = (1:length(data_nonfail));
wref_data_nonfail = data_nonfail(1:end,2);
w_meas_data_nonfail = data_nonfail(1:end,3);

w_meas_data_nonfail_mean = mean(w_meas_data_nonfail);

figure();
hold on;
plot(t_fail, wref_data_fail, DisplayName='GyY ref');
plot(t_fail, w_meas_data_fail, DisplayName='GyY');
grid on;
ylim([20 60])
xlabel('t')
ylabel('GyY(deg/s)')
title("K_{p} = 0.05, K_{i} = 0.0125, K_{d} = 0.05")
legend();


hold off;

figure();
hold on;
plot(t_nonfail, wref_data_nonfail, DisplayName='GyY ref');
plot(t_nonfail, w_meas_data_nonfail, DisplayName='GyY');
yline(w_meas_data_nonfail_mean, DisplayName="GyY mean")
grid on;
ylim([20 60])
xlabel('t')
ylabel('GyY(deg/s)')
title("K_{p} = 0.05, K_{i} = 0.0001, K_{d} = 0.05")
legend();


hold off;
disp(w_meas_data_nonfail_mean);