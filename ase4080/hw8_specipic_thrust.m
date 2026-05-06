clc; clear all;

Isp = 50:50:3000; %s
deltaV = 100; %m/s
m_dry = 150; %kg
g = 9.81; %m/s^2

N = length(Isp);
m_prop = zeros(1, N);
m_wet = zeros(1,N);

for i=1:1:N
    m_prop(i) = m_dry*(exp(deltaV/(Isp(i)*g))-1);
    m_wet(i) = m_prop(i)/(1-exp(-deltaV/(Isp(i)*g)));
end

figure;
subplot(2,1,1);
hold on;
plot(Isp, m_prop);
scatter(Isp, m_prop, DisplayName ='m_prop data')
xlabel('Isp[s]');
ylabel('m_{prop}[kg]');
grid('on');
hold off;

subplot(2,1,2);
hold on;
plot(Isp, m_wet);
scatter(Isp, m_wet, DisplayName ='m_wet data')
xlabel('Isp[s]');
ylabel('m_{wet}[kg]');
grid('on');
hold off;
