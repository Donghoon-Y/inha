clc; clear all;

%%cds1 cds2 ax ay az gx gy gz pf

data = readmatrix("two_ref_sunpointing.csv");

t = (1:length(data));
cds1_data = data(1:end,2);
cds2_data = data(1:end,3);
ax_data = data(1:end,4);
ay_data = data(1:end,5);
az_data = data(1:end,6);
gx_data = data(1:end,7);
gy_data = data(1:end,8);
gz_data = data(1:end,9);

gy_data_mean = mean(gy_data);




figure();
hold on;
plot(t, cds1_data, DisplayName="Cds1 Value");
plot(t, cds2_data, DisplayName="Cds2 Value");
yline(310, DisplayName="Cds1 ref", Color="red");
yline(360, DisplayName="Cds2 ref",Color = "Black");
xlabel("t");
ylabel('Cds');
title("Two cds experiment");
grid on;
legend show;

hold off;

figure();
hold on;
plot(t, gx_data, DisplayName="GyX");
plot(t, gy_data, DisplayName="GyY");
plot(t, gz_data, DisplayName="GyZ");
yline(gy_data_mean, DisplayName="GyY mean", color= 'black');
xlabel("t");
ylabel('GyY');
title("Gyro Y value according to Two cds experiment");
grid on;
legend show;

hold off;

disp(gy_data_mean);