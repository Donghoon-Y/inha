import yfinance as yf
import datetime as dt
import numpy as np
from matplotlib import pyplot as plt
import scipy.sparse as ssp
import scipy.sparse.linalg as sla
import cvxpy as cp

end = dt.datetime.today()
start = dt.datetime(end.year - 10, end.month, end.day)

df =yf.download('005930.KS',start, end)
'''
#정규화하기 전 Plot
plt.figure(figsize=(14,9), dpi=100)
plt.plot(df['Close'], label='Samsung Electronics')
plt.xlabel('date')
plt.ylabel('stock price')
plt.grid()
plt.legend()
plt.show()
'''
#기존 data의 y의 수적오류를 줄이기 위해서 정규화 진행 후 plot
y = df['Close'].values
y -= np.mean(y)
y /= np.std(y)

y = y.flatten()

n = len(y)
'''
plt.figure(figsize=(14,9), dpi=100)
plt.plot(y, label='Samsung Electronics')
plt.xlabel('days')
plt.ylabel('normalized price')
plt.grid()
plt.legend()
plt.show()
'''
lam = 1e5

A = np.eye(n)
D = np.zeros((n-2,n))
for i in range(D.shape[0]):
  D[i,i:i+3] = [1, -2, 1]

''' 
#numpy의 라이브러리를 이용해서 해결 -> 좀 느리다.
A_tilde = np.vstack((A,np.sqrt(lam)*D))
y_tilde = np.hstack((y,np.zeros(n-2)))

x_hat = np.linalg.lstsq(A_tilde, y_tilde)[0]

plt.plot(y, label='Samsung Electronics')
plt.plot(x_hat, linewidth=3, label='optimal trend filter')
plt.xlabel('days')
plt.ylabel('normalized price')
plt.grid()
plt.legend()
plt.show()
'''
#A와 D matrix가 sparse 한 점을 이용하여 scipy를 통해서 계산하면 효율적이게 연산 가능
As = ssp.eye(n)
Ds = ssp.spdiags([np.ones(n), -2*np.ones(n), np.ones(n)], [0,1,2],n-2,n)

As_tilde = ssp.vstack((As, np.sqrt(lam)*Ds))
y_tilde = np.hstack((y,np.zeros(n-2)))

x_hat = sla.lsqr(As_tilde, y_tilde)[0]

plt.plot(y, label='Samsung Electronics')
plt.plot(x_hat, linewidth=3, label='optimal trend filter')
plt.xlabel('days')
plt.ylabel('normalized price')
plt.grid()
plt.legend()
plt.show()

#1-norm을 이용하여 least square가 아니라 다른 방식으로 해결 - > 더 좋은 결과
x = cp.Variable(n)
obj = cp.Minimize( cp.norm(y-As@x,1) + 100*cp.norm(Ds@x,1) )
prob = cp.Problem(obj)
prob.solve(solver=cp.ECOS, verbose=False, max_iters=100)

plt.figure(figsize=(14,9), dpi=100)
plt.plot(y, alpha=0.4, label='Samsung Electronics')
plt.plot(x_hat,linewidth=4, alpha=0.6, label=r'$\ell_2$ trend filter')
plt.plot(x.value,linewidth=3, label=r'$\ell_1$ trend filter')
plt.xlabel('days')
plt.ylabel('normalized price')
plt.grid()
plt.legend()
plt.show()

#y의 data에 1%의 확률로 noise가 발생한다고 가정
z = 10*np.random.randn(n)

for i in range(n):
  if np.random.rand() > 0.01: #99%가 충족해서 1%의 노이즈가 발생하는 것을 구현
    z[i] = 0

y_cor = y + z

#2-norm 즉 least square방식으로 해결했을 때의 값
b_tilde_cor = np.hstack((y_cor, np.zeros(n-2)))
xhats_cor = sla.lsqr(As_tilde,b_tilde_cor)[0]

plt.figure(figsize=(14,9), dpi=100)
plt.plot(y_cor, alpha=0.4, label='Samsung Electronics (corrupted)')
plt.plot(x_hat,linewidth=4, alpha=0.6, label=r'$\ell_2$ trend filter (on clean data)')
plt.plot(xhats_cor,linewidth=4, alpha=0.6, label=r'$\ell_2$ trend filter (on corrupted data)')
plt.xlabel('days')
plt.ylabel('normalized price')
plt.ylim(-1.5,3.5)
plt.grid()
plt.legend()
plt.show()

#1-norm으로 했을 때의 값 - > Noise가 있어도 정확한 결과값을 낸다. 단, 노이즈가 너무 많으면 안됨
x_cor = cp.Variable(n)
obj = cp.Minimize( cp.norm(y_cor-As@x_cor,1) + 100*cp.norm(Ds@x_cor,1) )
prob = cp.Problem(obj)
prob.solve(solver=cp.ECOS, verbose=False, max_iters=100)

plt.figure(figsize=(14,9), dpi=100)
plt.plot(y_cor, alpha=0.4, label='Samsung Electronics')
plt.plot(x.value, linewidth=4, label=r'$\ell_1$ trend filter (on clean data)')
plt.plot(x_cor.value, linewidth=3, label=r'$\ell_1$ trend filter (on corrupted data)')
plt.xlabel('days')
plt.ylabel('normalized price')
plt.ylim(-1.5,3.5)
plt.grid()
plt.legend()
plt.show()