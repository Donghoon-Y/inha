import numpy as np
import matplotlib.pyplot as plt

# 실험에 사용할 R 값들
R_list = [0.01, 0.1, 1.0, 10.0]

# 시스템 설정
a, b = 2, -0.5
A = np.array([[a, 1], [b, 0]])
H = np.array([[1, 0]])
Q = 0.01 * np.eye(2)
P0 = 0.1 * np.eye(2)
x0 = np.array([[1.0], [0.0]])

# 저장용 리스트
alpha_err_all = []
sigma_alpha_all = []
trace_P_all = []

# 반복
for R in R_list:
    np.random.seed(0)
    x_true = x0.copy()
    x_hat = x0.copy()
    P = P0.copy()

    alpha_err_list = []
    sigma_alpha_list = []
    trace_P_list = []

    for k in range(20):
        # 실제 상태 전파
        w_k = np.random.multivariate_normal(mean=[0, 0], cov=Q).reshape(2, 1)
        x_true = A @ x_true + w_k

        # 측정
        v_k = np.random.normal(0, np.sqrt(R))
        y_k = H @ x_true + v_k

        # 예측
        x_pred = A @ x_hat
        P_pred = A @ P @ A.T + Q

        # 업데이트
        S = H @ P_pred @ H.T + R
        K = P_pred @ H.T @ np.linalg.inv(S)
        x_hat = x_pred + K @ (y_k - H @ x_pred)
        P = (np.eye(2) - K @ H) @ P_pred @ (np.eye(2) - K @ H).T + K @ K.T*R

        # 저장
        alpha_err_list.append(x_hat[0, 0] - x_true[0, 0])
        sigma_alpha_list.append(np.sqrt(P[0, 0]))
        trace_P_list.append(np.trace(P))

    alpha_err_all.append(alpha_err_list)
    sigma_alpha_all.append(sigma_alpha_list)
    trace_P_all.append(trace_P_list)

# === 그래프 1: Alpha 오차 ±3σ ===
fig, axs = plt.subplots(2, 2, figsize=(12, 8))
axs = axs.flatten()

for i, R in enumerate(R_list):
    axs[i].plot(alpha_err_all[i], label='Alpha Error')
    axs[i].plot(3*np.array(sigma_alpha_all[i]), 'r--', label='+3σ')
    axs[i].plot(-3*np.array(sigma_alpha_all[i]), 'r--', label='-3σ')
    axs[i].set_title(f'R = {R}')
    axs[i].set_xlabel('Time Step (k)')
    axs[i].set_ylabel('Error')
    axs[i].grid(True)
    axs[i].legend(loc='upper right')

plt.suptitle("Alpha Estimation Error and ±3σ Bounds for Different R", fontsize=14)
plt.tight_layout(rect=[0, 0, 1, 0.95])
plt.show()

# === 그래프 2: Trace(P) 비교 ===
plt.figure(figsize=(10, 5))
for i, R in enumerate(R_list):
    plt.plot(trace_P_all[i], label=f'R = {R}')

plt.title("Trace(P) Over Time for Different R Values")
plt.xlabel("Time Step (k)")
plt.ylabel("Trace(P)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
