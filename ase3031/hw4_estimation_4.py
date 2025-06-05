import numpy as np
from matplotlib import pyplot as plt

np.random.seed(0)
theta_true = np.array([2, -0.5])
A = np.array([[theta_true[0], 1],[theta_true[1], 0]])
H = np.array([1,0]).T
R = 0.01
P0 = 0.1*np.eye(2)
Q = 0.01 * np.eye(2)

xhat0 = np.array([[1],[0]])
x0 = xhat0 #x0의 초기조건이 존재하지않아 같다고 가정하고 시작함.
xk = [x0] #실제값
yk = []
xhat = [xhat0]
Pk = [P0]
tracePk = [np.trace(P0)]

#동역학?모델과 초기 추정치를 이용하여 추정치 전파
for i in range(0,21) :
    noise = np.random.multivariate_normal(mean=[0, 0], cov=Q).reshape(2,1)
    xk_cal = A@xk[i] + noise
    xk.append(xk_cal) #실제 x값 전파
for i in range(0,len(xk)) :
    yk_cal = H@xk[i] + np.random.normal(0, np.sqrt(R))
    yk.append(yk_cal) #실제 측정치 전파

x_hat_history = []
P_history = []
P_trace_history = []

for i in range(len(xk)) :
    xk_cal = A@xhat[i] 
    Pk_cal = A@Pk[i]@A.T + Q
    xhat.append(xk_cal)
    Pk.append(Pk_cal)

    #칼만게인을 이용하여 수치 업데이트 
    Kk = Pk_cal@H.T/(R+H@Pk_cal@H.T)
    x_hat_update = xk_cal + Kk*(yk[i] - H@xk_cal)
    Pk_update = (np.eye(2)-Kk@H) @ Pk_cal @ (np.eye(2)-Kk@H).T + (Kk*R)@ Kk.T
    P_history.append(Pk_update)
    P_trace_history.append(np.trace(Pk_update))
    x_hat_history.append(x_hat_update)

x_hat1 = []
x_hat2 = []
sigma1 = []
sigma2 = []

for x in x_hat_history :
    x1_cal = x[0,0]
    x2_cal = x[1,0]
    x_hat1.append(x1_cal)
    x_hat2.append(x2_cal)

for P in P_history :
    sigma1_cal = np.sqrt(P[0,0])
    sigma2_cal = np.sqrt(P[1,1])
    sigma1.append(sigma1_cal)
    sigma2.append(sigma2_cal)

time = range(len(x_hat1))
print(H.T)

plt.plot(time, x_hat1, 'x-',label='Estimated x1')
plt.fill_between(time, np.array(x_hat1) - 3 * np.array(sigma1), np.array(x_hat1) + 3 * np.array(sigma1), alpha=0.3, label='±3σ interval')
plt.title("State Estimate x1 with ±3σ Confidence Interval")
plt.xlabel("Time Step k")
plt.ylabel("x1")
plt.legend()
plt.grid(True)
plt.show()

plt.plot(time, x_hat2, 'x-',label='Estimated x2')
plt.fill_between(time,np.array(x_hat2) - 3 * np.array(sigma2), np.array(x_hat2) + 3 * np.array(sigma2), alpha=0.3, label='±3σ interval')
plt.title("State Estimate x2 with ±3σ Confidence Interval")
plt.xlabel("Time Step k")
plt.ylabel("x2")
plt.legend()
plt.grid(True)
plt.show()


plt.plot(P_trace_history, 'o--')
plt.title("Trace of Pk (Total Uncertainty Over Time)")
plt.xlabel("Time Step k")
plt.ylabel("trace(Pk)")
plt.grid(True)
plt.show()

