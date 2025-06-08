import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# 데이터 정의
X = np.array([
    [1, 0],
    [0, 1],
    [1, 1],
    [2, 1],
    [3, 0]
])
y = np.array([2, -1, 1.1, 2.2, 6.1])

# 초기 상태
theta_init = np.array([[10.0], [10.0]]) 
theta_init_2 = np.array([[0.0], [0.0]]) 
P_init = np.eye(2) 
P_init_2 = 10**6 * np.eye(2)
R = 1.0
batch_theta0 = np.array([1.9583, -1.192])

# RLS 함수 정의
def recursive_least_squares(X, y, theta, P, R):
    theta_hat = []
    P_update = []

    for k in range(len(y)):
        x_k = X[k].reshape(2, 1)
        y_k = y[k]

        K = P @ x_k / (x_k.T @ P @ x_k + R)
        y_pred = x_k.T @ theta
        theta = theta + K * (y_k - y_pred)
        P = P - K @ x_k.T @ P

        theta_hat.append([theta[0, 0], theta[1, 0]])
        P_update.append([P[0, 0], P[0, 1], P[1, 0], P[1, 1]])

    return np.array(theta_hat), np.array(P_update)

# 실행
theta_history, P_diag_history = recursive_least_squares(X, y, theta_init, P_init, R)
theta_history_2, P_diag_history_2 = recursive_least_squares(X, y, theta_init_2, P_init_2, R)

# 시각화
steps = np.arange(1, len(y) + 1)
plt.figure(figsize=(12, 5))

# θ₀
plt.subplot(1, 2, 1)
plt.plot(steps, theta_history[:, 0], 'o-', label='theta0_init(10,10)')
plt.plot(steps, theta_history_2[:, 0], 'x-', label='theta0_init(0,0)')
plt.axhline(y=batch_theta0[0], color='r', linestyle='--', label='Batch θ₀')
plt.xlabel("Update Step")
plt.ylabel("Parameter Estimate")
plt.title("Updated Parameters θ₀")
plt.grid(True)
plt.legend()

# θ₁
plt.subplot(1, 2, 2)
plt.plot(steps, theta_history[:, 1], 's-', label='theta1_init(10,10)')
plt.plot(steps, theta_history_2[:, 1], 'd-', label='theta1_init(0,0)')
plt.axhline(y=batch_theta0[1], color='r', linestyle='--', label='Batch θ₁')
plt.xlabel("Update Step")
plt.ylabel("Parameter Estimate")
plt.title("Updated Parameters θ₁")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()

# 보기 좋게 출력
np.set_printoptions(precision=4, suppress=True)

# 모든 θ 값 출력
df_theta1 = pd.DataFrame(theta_history, columns=["θ₀ (init 10)", "θ₁ (init 10)"])
df_theta2 = pd.DataFrame(theta_history_2, columns=["θ₀ (init 0)", "θ₁ (init 0)"])

# 모든 공분산 행렬 요소 출력
df_P1 = pd.DataFrame(P_diag_history, columns=["P₀₀", "P₀₁", "P₁₀", "P₁₁"])
df_P2 = pd.DataFrame(P_diag_history_2, columns=["P₀₀", "P₀₁", "P₁₀", "P₁₁"])

print("\n[2.3] θ 추정값 (초기값 10,10 기준):")
print(df_theta1)

print("\n[2.3] 공분산 행렬 P (초기값 10,10 기준):")
print(df_P1)

print("\n[2.2] θ 추정값 (초기값 0,0 기준):")
print(df_theta2)

print("\n[2.2] 공분산 행렬 P (초기값 0,0 기준):")
print(df_P2)
