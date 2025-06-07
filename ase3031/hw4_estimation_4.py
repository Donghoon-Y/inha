import numpy as np
import matplotlib.pyplot as plt

# 설정값
a = 2
b = -0.5

A = np.array([[a, 1],
              [b, 0]])
Q = 0.01 * np.eye(2)

H = np.array([[1, 0]])
R = 0.05

# 초기 상태
x_true = np.array([[1.0],
                   [0.0]])

x_hat = np.array([[1.0],
                  [0.0]])

P = 0.1 * np.eye(2)

N = 20  # 총 스텝 수

# 저장용 리스트 초기화
x_true_list = []
x_hat_list = []
error1_list = []
error2_list = []
sigma1_list = []
sigma2_list = []
trace_P_list = []

# 난수 고정
np.random.seed(0)

# 메인 루프
for k in range(N):
    # --- 실제 상태 전파 ---
    w_k = np.random.multivariate_normal(mean=[0, 0], cov=Q).reshape(2,1)
    x_true = A @ x_true + w_k
    x_true_list.append(x_true.flatten())

    # --- 측정값 생성 ---
    v_k = np.random.normal(0, np.sqrt(R))
    y_k = H @ x_true + v_k

    # --- 예측 단계 ---
    x_pred = A @ x_hat
    P_pred = A @ P @ A.T + Q

    # --- 업데이트 단계 ---
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)
    x_hat = x_pred + K @ (y_k - H @ x_pred)
    P = (np.eye(2) - K @ H) @ P_pred @ (np.eye(2) - K @ H).T + K @ K.T * R

    # --- 저장 ---
    x_hat_list.append(x_hat.flatten())
    error1_list.append((x_hat[0,0] - x_true[0,0]))
    error2_list.append((x_hat[1,0] - x_true[1,0]))
    sigma1_list.append(np.sqrt(P[0,0]))
    sigma2_list.append(np.sqrt(P[1,1]))
    trace_P_list.append(np.trace(P))

# numpy 변환
error1_array = np.array(error1_list)
error2_array = np.array(error2_list)
sigma1_array = np.array(sigma1_list)
sigma2_array = np.array(sigma2_list)
trace_array = np.array(trace_P_list)

# --- 시각화 ---

plt.figure(figsize=(10,5))
plt.plot(error1_array, label="Alpha Estimation Error")
plt.plot(3*sigma1_array, 'r--', label="+3σ")
plt.plot(-3*sigma1_array, 'r--', label="-3σ")
plt.title("Alpha Estimation Error with ±3σ Bounds")
plt.xlabel("Time Step k")
plt.grid(True)
plt.legend()
plt.show()

plt.figure(figsize=(10,5))
plt.plot(error2_array, label="Alpha Estimation Error")
plt.plot(3*sigma2_array, 'r--', label="+3σ")
plt.plot(-3*sigma2_array, 'r--', label="-3σ")
plt.title("Alpha Estimation Error with ±3σ Bounds")
plt.xlabel("Time Step k")
plt.grid(True)
plt.legend()
plt.show()

plt.figure(figsize=(8,4))
plt.plot(trace_array, marker='o', label="Trace(P)")
plt.title("Trace of Covariance Matrix P over Time")
plt.xlabel("Time Step k")
plt.ylabel("Trace(P)")
plt.grid(True)
plt.legend()
plt.show()

diff = np.abs(np.diff(trace_P_list))
print("마지막 Trace(Pk) 변화량:", diff[-5:])
