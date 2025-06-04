import numpy as np
from matplotlib import pyplot as plt

np.random.seed(0)  # For reproducibility

k = range(2,22,1)
theta_true = np.array([2, -0.5])

uk = []
psi = []
yk = []
for i in k :
    uk_cal = np.sin(0.2*(i-1)) + 0.1*np.random.normal(0, 0.1)
    uk.append(uk_cal)

for i in range(1, len(k)) :
    yk_cal = np.array([uk[i], uk[i-1]]).T @ theta_true + np.random.normal(0, 0.01)
    yk.append(yk_cal)

for i in range(1,len(k)) :
    psi_cal = np.array([uk[i], uk[i-1]])
    psi.append(psi_cal)

theta_hat = np.linalg.inv(np.array(psi).T @ np.array(psi)) @ np.array(psi).T @ yk
print("Estimated Parameters:", theta_hat)

A = np.array([[theta_hat[0], 1],[theta_hat[1], 0]])

P0 = 0.1*np.eye(2)
Q = 0.01 * np.eye(2)
xhat0 = np.array([[1],[0]])
xk = [xhat0]
Pk = [P0]

for i in range(0,21) :
    xk_cal = A@xk[i]
    xk.append(xk_cal)
    Pk_cal = A@Pk[i]@A.T + Q
    Pk.append(Pk_cal)

