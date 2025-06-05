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
#3.1 theta hat을 least square 값을 이용해서 구한다.
theta_hat = np.linalg.inv(np.array(psi).T @ np.array(psi)) @ np.array(psi).T @ yk
print("Estimated Parameters:", theta_hat)

#상태 전파를 위한 행렬을 theta hat을 이용하여 구성한다
A = np.array([[theta_hat[0], 1],[theta_hat[1], 0]])
A_true = np.array([[2, 1], [-0.5, 0]])  # 참값

P0 = 0.1*np.eye(2)
Q = 0.01 * np.eye(2)
xhat0 = np.array([[1],[0]])
xtrue0 = xhat0.copy()
x_true = [xtrue0]
xk = [xhat0]
Pk = [P0]
tracePk = [np.trace(P0)]

#동역학?모델과 초기 추정치를 이용하여 추정치 전파
for i in range(0,21) :
    wk = np.random.multivariate_normal(np.zeros(2), Q).reshape(2, 1)
    x_true_next = A_true @ x_true[-1] + wk
    x_true.append(x_true_next)
    xk_cal = A@xk[i]
    xk.append(xk_cal)
    Pk_cal = A@Pk[i]@A.T + Q
    Pk.append(Pk_cal)
    tracePk.append(np.trace(Pk_cal))
x1 = []
x2 = []
sigma1 = []
sigma2 = []
for x in xk :
    x1_cal = x[0,0]
    x2_cal = x[1,0]
    x1.append(x1_cal)
    x2.append(x2_cal)

for P in Pk :
    sigma1_cal = np.sqrt(P[0,0])
    sigma2_cal = np.sqrt(P[1,1])
    sigma1.append(sigma1_cal)
    sigma2.append(sigma2_cal)

x1_true = [x[0, 0] for x in x_true]
x2_true = [x[1, 0] for x in x_true]

error1 = (np.array(x1) - np.array(x1_true))
error2 = (np.array(x2) - np.array(x2_true))

time = range(len(x1))

#3.3 컴퓨터로 증명 
eigenvalue = np.linalg.eigvals(A)
print(f'eigenvalue : {eigenvalue}')

plt.plot(time, error1, 'x-',label='Estimated error x1')
plt.fill_between(time,- 3 * np.array(sigma1), + 3 * np.array(sigma1), alpha=0.3, label='±3σ interval')
plt.title("State Estimate x1 with ±3σ Confidence Interval")
plt.xlabel("Time Step k")
plt.ylabel("Error x1")
plt.legend()
plt.grid(True)
plt.show()

plt.plot(time, error2, 'x-',label='Estimated error x2')
plt.fill_between(time, - 3 * np.array(sigma2), + 3 * np.array(sigma2), alpha=0.3, label='±3σ interval')
plt.title("State Estimate x2 with ±3σ Confidence Interval")
plt.xlabel("Time Step k")
plt.ylabel("Error x2")
plt.legend()
plt.grid(True)
plt.show()

plt.plot(tracePk, 'o--')
plt.title("Trace of Pk (Total Uncertainty Over Time)")
plt.xlabel("Time Step k")
plt.ylabel("trace(Pk)")
plt.grid(True)
plt.show()