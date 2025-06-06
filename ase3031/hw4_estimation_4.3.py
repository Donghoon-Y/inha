import numpy as np
from matplotlib import pyplot as plt

R_list = [0.01, 0.1, 1.0, 10.0]
trace_results = {}

for R in R_list:
    np.random.seed(0)
    theta_hat = np.array([1.88351422, -0.36621749])
    A = np.array([[theta_hat[0], 1], [theta_hat[1], 0]])
    H = np.array([[1, 0]])
    P0 = 0.1 * np.eye(2)
    Q = 0.01 * np.eye(2)

    xhat0 = np.array([[1], [0]])
    x0 = xhat0.copy()
    xk = [x0]
    yk = []
    xhat = [xhat0]
    Pk = [P0]

    for i in range(0, 21):
        noise = np.random.multivariate_normal(mean=[0, 0], cov=Q).reshape(2, 1)
        xk_cal = A @ xk[i] + noise
        xk.append(xk_cal)

    for i in range(len(xk)):
        yk_cal = H @ xk[i] + np.random.normal(0, np.sqrt(R))
        yk.append(yk_cal)

    P_trace_history = []

    for i in range(len(xk)):
        x_pred = A @ xhat[i]
        P_pred = A @ Pk[i] @ A.T + Q

        Kk = P_pred @ H.T / (R + H @ P_pred @ H.T)
        residual = yk[i] - H @ x_pred
        x_upd = x_pred + Kk @ residual
        I = np.eye(2)
        P_upd = (I - Kk @ H) @ P_pred @ (I - Kk @ H).T + R * Kk @ Kk.T

        xhat.append(x_upd)
        Pk.append(P_upd)
        P_trace_history.append(np.trace(P_upd))

    trace_results[f"R={R}"] = P_trace_history
print(trace_results)
for label, trace in trace_results.items():
    plt.plot(trace, label=label)

plt.title("Trace(Pk) Over Time for Different R values")
plt.xlabel("Time Step k")
plt.ylabel("Trace(Pk)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
