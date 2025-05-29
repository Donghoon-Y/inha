% 시스템 정의 (PD 제어기 포함)
s = tf('s');
G = (s + 1) / (s^2 * (s - 0.1));
C = (2*s + 1);   % PD controller
L = C * G;

% Root Locus 및 감쇠비선 그리기
figure
rlocus(L)
sgrid(1/sqrt(2), [],color ='b')   % 감쇠비 = 1/sqrt(2)
title('Root Locus with Damping Ratio Line (ζ = 1/√2)')
grid on

% 마우스로 원하는 극점 위치 클릭해서 Gain 찾기
[K, poles] = rlocfind(L);  % 클릭하면 해당 극점에 대응하는 K 반환

disp(['선택된 K: ', num2str(K)])
disp('폐루프 극점:')
disp(poles)
