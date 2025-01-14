import numpy as np
from scipy.integrate import solve_ivp
from pyquaternion import Quaternion
import matplotlib.pyplot as plt

# 위성 기본 파라미터 설정
class Satellite:
    def __init__(self):
        # 위성 질량 및 관성 모멘트
        self.mass = 4.0  # kg
        self.inertia = np.diag([0.02, 0.02, 0.01])  # kg·m²
        self.inv_inertia = np.linalg.inv(self.inertia)

        # 초기 상태 (위치, 속도, 자세)
        self.position = np.array([7000, 0, 0])  # km
        self.velocity = np.array([0, 7.5, 0])  # km/s
        self.attitude = Quaternion(axis=[1, 0, 0], angle=0)  # 초기 쿼터니언
        self.angular_velocity = np.array([0, 0, 0])  # rad/s

    def dynamics(self, t, state, control_torque):
        """위성의 운동 방정식"""
        # 상태 분리
        position = state[:3]
        velocity = state[3:6]
        quat = Quaternion(state[6:10])  # 쿼터니언
        omega = state[10:]  # 각속도
        
        # 궤도 동역학 (단순화)
        r = np.linalg.norm(position)
        accel = -398600.4418 / r**3 * position  # 중력 가속도 (km/s²)
        
        # 자세 동역학
        quat_dot = 0.5 * Quaternion(0, *omega) * quat
        torque = control_torque(t)  # 제어 입력
        omega_dot = np.dot(self.inv_inertia, torque - np.cross(omega, np.dot(self.inertia, omega)))

        # 미분 방정식 반환
        return np.concatenate([velocity, accel, quat_dot.elements, omega_dot])

    def simulate(self, t_span, dt, control_torque):
        """위성 시뮬레이션"""
        # 초기 상태
        initial_state = np.concatenate([
            self.position,
            self.velocity,
            self.attitude.elements,
            self.angular_velocity
        ])

        # 시간 범위 및 적분
        t_eval = np.arange(t_span[0], t_span[1], dt)
        result = solve_ivp(
            fun=lambda t, y: self.dynamics(t, y, control_torque),
            t_span=t_span,
            y0=initial_state,
            t_eval=t_eval,
            method='RK45'
        )

        return result

# 제어 토크 함수 (예제: 태양 포인팅 제어)
def sun_pointing_control(t):
    # 간단한 PID 제어 예시
    return np.array([0.0, 0.0, 0.0])

# 시뮬레이션 실행
satellite = Satellite()
t_span = (0, 6000)  # 0초부터 6000초까지
dt = 1.0  # 시간 간격

result = satellite.simulate(t_span, dt, sun_pointing_control)

# 결과 시각화
positions = result.y[:3, :].T
plt.figure(figsize=(10, 6))
plt.plot(positions[:, 0], positions[:, 1])
plt.title('Satellite Orbit Simulation')
plt.xlabel('X Position (km)')
plt.ylabel('Y Position (km)')
plt.grid()
plt.axis('equal')
plt.show()
