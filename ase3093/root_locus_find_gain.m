% 시스템 정의 (PD 제어기 포함)
s = tf('s');
G = (s + 1) / (s^2 * (s - 0.1));
C = (2*s + 1);   % PD controller
L = C * G;

% Root Locus 그리기
figure
rlocus(L)
hold on

% 감쇠비 ζ = 1/√2 에 해당하는 선을 직접 그림 (파란색 dashed)
zeta = 1 / sqrt(2);         % 감쇠비
theta = acos(zeta);         % 각도 (radian)

r = linspace(0, 5, 100);    % 반지름 (거리)

% 감쇠비 선 좌표 계산
x = -r * cos(theta);
y =  r * sin(theta);
plot(x,  y, 'b--', 'LineWidth', 1.5)  % 위쪽 선 (positive imag)
plot(x, -y, 'b--', 'LineWidth', 1.5)  % 아래쪽 선 (negative imag)

title('Root Locus with Damping Ratio Line (ζ = 1/√2, blue dashed)')
xlabel('Real Axis')
ylabel('Imaginary Axis')
grid on

% 루프를 통해 반복 선택 기능 추가
while true
    disp('Root Locus에서 극점을 클릭하세요 (창을 닫거나 Esc 누르면 종료됩니다)')
    try
        [K, poles] = rlocfind(L);  % 마우스로 클릭하여 K 및 극점 선택
        disp(['선택된 K: ', num2str(K)])
        disp('폐루프 극점:')
        disp(poles)
    catch
        disp('루트 로커스 선택이 종료되었습니다.')
        break
    end
end
