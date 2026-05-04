clc; clear all;

Gt = 0; %dBi
P = 1; %W -> log로 변환해야 한다. 
Ll = 0;
epsilon_t = 10; %deg
theta_t = 30; %deg
epsilon_r = 1; %deg
theta_r = 4; %deg
c = 299792458; %m/s
d = 22*10^5; %m
f = 435*10^6; %Hz
D = 2.4;
eta = 0.6;
Ts = 300;
R = 64*10^3; %bps

EIRP = log10(P) + Gt + Ll;
Lpt = -12*(epsilon_t/theta_t)^2;
Lpr = -12*(epsilon_r/theta_r)^2;
Ls = 20*log10(c) - 20*log10(4*pi) - 20*log10(d) - 20*log10(f);
Gr = -159.59 + 20*log10(D) + 20*log10(f) + 10*log10(eta);
G_T_dB_K = Gr/Ts;
G_T_dBi = Gr - 10*log10(Ts);
ideal_Eb_N0 = EIRP +Lpr + Lpt + Ls+ G_T_dBi +228.6 - 10*log10(R);
LM = ideal_Eb_N0 - 7;
fprintf("EIRP : %f\n", EIRP);
fprintf("Lpt : %f\n", Lpt);
fprintf("Ls : %f\n", Ls);
fprintf("Lpr : %f\n", Lpr);
fprintf("Gr : %f\n", Gr);
fprintf("G/T : %f\n", G_T_dB_K);
fprintf("Eb/N0 : %f\n", ideal_Eb_N0);
fprintf("LM : %f\n", LM);