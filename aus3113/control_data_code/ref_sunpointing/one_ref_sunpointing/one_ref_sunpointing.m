clc; clear all;

data = readmatrix("one_ref_sunpointing_log.csv");

t = (1:length(data)-1);
cds1_data = data(2:end,2);


figure();
hold on;
plot(t, cds1_data, DisplayName="Cds1 Value");
yline(310, DisplayName="Cds1 ref");
xlabel("t");
ylabel('Cds1');
title("One cds experiment");
grid on;
legend show;

hold off;
