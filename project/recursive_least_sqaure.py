import numpy as np
import matplotlib.pyplot as plt

# === 설정 ===
n_steps = 50  # 시뮬레이션 스텝 수
true_x = np.array([10, 5])  # 실제 상태
x_hat = np.array([8, 7], dtype=float)  # 초기 추정
P = np.diag([1, 1])  # 초기 공분산
R = 0.01  # 측정 잡음 분산
sigma = np.sqrt(R)

# 결과 저장용
x_hat_history = []
P_trace_history = []

for k in range(1, n_steps + 1):
    # H_k = [1, 0.99^(k-1)]
    H_k = np.array([[1, 0.99**(k-1)]])
    
    # y_k = H_k x + v
    noise = np.random.normal(0, sigma)
    y_k = H_k @ true_x + noise  # 스칼라 측정값

    # 칼만 이득 K_k
    S_k = H_k @ P @ H_k.T + R  # 스칼라 (1x1)
    K_k = P @ H_k.T / S_k  # (2x1)

    # 상태 추정 갱신
    residual = y_k - H_k @ x_hat
    x_hat = x_hat + K_k.flatten() * residual

    # 공분산 갱신
    P = (np.eye(2) - K_k @ H_k) @ P @ (np.eye(2) - K_k@H_k).T + (K_k*R)@K_k.T

    # 기록
    x_hat_history.append(x_hat.copy())
    P_trace_history.append(np.trace(P))

# numpy 배열로 변환
x_hat_history = np.array(x_hat_history)

# === 결과 시각화 ===
plt.figure(figsize=(10, 5))
plt.plot(x_hat_history[:, 0], label='Estimated x1')
plt.plot(x_hat_history[:, 1], label='Estimated x2')
plt.hlines(true_x[0], 0, n_steps, colors='gray', linestyles='dashed', label='True x1')
plt.hlines(true_x[1], 0, n_steps, colors='gray', linestyles='dotted', label='True x2')
plt.xlabel('Time step')
plt.ylabel('State estimates')
plt.title('Kalman Filter Estimation of x1 and x2')
plt.legend()
plt.grid()
plt.show()
