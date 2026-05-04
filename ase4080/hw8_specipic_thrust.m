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
plot(Isp, m_prop);
xlabel('Isp');
ylabel('m_{prop}');
grid('on');

subplot(2,1,2);
plot(Isp, m_wet);
xlabel('Isp');
ylabel('m_{wet}');
grid('on');
