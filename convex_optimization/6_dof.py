import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# 위성 및 리액션 휠 파라미터
mass = 500  # kg
I = np.diag([100, 100, 50])  # 관성 모멘트 (kg·m^2)
I_rw = 0.01  # 리액션 휠 관성 (kg·m^2)
rw_saturation = 3000  # 리액션 휠 최대 각속도 (rad/s)

# 제어 파라미터
Kp = 1.0  # 비례 제어 이득
Kd = 0.1  # 미분 제어 이득

# 태양 벡터 계산 (고정된 예제)
sun_vector = np.array([1, 0, 0])  # 태양 방향 (단위 벡터)

def satellite_dynamics(t, state):
    r = state[:3]  # 위치 벡터
    v = state[3:6]  # 속도 벡터
    q = state[6:10]  # 쿼터니언
    omega = state[10:13]  # 각속도 벡터
    rw_speed = state[13:16]  # 리액션 휠 각속도
    
    # 쿼터니언에서 자세 매트릭스 계산
    q0, q1, q2, q3 = q
    C_bi = np.array([
        [1 - 2*(q2**2 + q3**2), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
        [2*(q1*q2 + q0*q3), 1 - 2*(q1**2 + q3**2), 2*(q2*q3 - q0*q1)],
        [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*(q1**2 + q2**2)]
    ])
    
    # 태양 벡터와 목표 자세 계산
    sun_body = C_bi.dot(sun_vector)
    error = np.cross(sun_body, np.array([1, 0, 0]))  # 목표 자세: x축 태양 정렬
    
    # 자세 제어 토크 계산 (PD 제어)
    torque = Kp * error - Kd * omega
    
    # 리액션 휠 포화 처리
    rw_torque = np.clip(torque, -rw_saturation * I_rw, rw_saturation * I_rw)
    rw_accel = rw_torque / I_rw
    omega_dot = np.linalg.inv(I).dot(torque - np.cross(omega, I.dot(omega)))
    
    # 쿼터니언 미분 계산
    q_dot = 0.5 * np.array([
        [-q1, -q2, -q3],
        [ q0, -q3,  q2],
        [ q3,  q0, -q1],
        [-q2,  q1,  q0]
    ]).dot(omega)
    
    return np.hstack((v, np.zeros(3), q_dot, omega_dot, rw_accel))

# 초기 조건
r0 = np.array([7000, 0, 0])  # 초기 위치
v0 = np.array([0, 7.5, 0])  # 초기 속도
q0 = np.array([1, 0, 0, 0])  # 초기 쿼터니언
omega0 = np.array([0, 0, 0])  # 초기 각속도
rw_speed0 = np.array([0, 0, 0])  # 초기 리액션 휠 속도

state0 = np.hstack((r0, v0, q0, omega0, rw_speed0))
t_span = (0, 5400)  # 시뮬레이션 시간
t_eval = np.linspace(*t_span, 1000)

# 시뮬레이션 실행
solution = solve_ivp(satellite_dynamics, t_span, state0, t_eval=t_eval, method='RK45')

# 결과 시각화
q_sol = solution.y[6:10, :].T
omega_sol = solution.y[10:13, :].T

plt.plot(t_eval, omega_sol)
plt.xlabel("Time (s)")
plt.ylabel("Angular Velocity (rad/s)")
plt.title("Satellite Angular Velocity")
plt.legend(["wx", "wy", "wz"])
plt.show()
