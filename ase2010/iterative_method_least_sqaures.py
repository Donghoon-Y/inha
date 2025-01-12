import numpy as np
from matplotlib import pyplot as plt

np.random.seed(0)
m, n = 20, 10
A = np.random.randn(m, n)
b = np.random.randn(m)

x_hat = np.linalg.pinv(A)@b #np.linalg.pinv는 seudo inverse를 구해준다.

mu = 1/np.linalg.norm(A)**2
x_k = np.zeros(n)
iteration = 500

K = []
E = []
for i in range(iteration) :
    x_k = x_k - mu*A.T@(A@x_k-b)
    error = np.linalg.norm(x_k-x_hat)
    E.append(error)
    K.append(i)


plt.plot(K, E)
plt.xlabel('Iteration (k)')
plt.ylabel(r'$\|x^{(k)} - \hat{x}\|$')
plt.title(r'Convergence of $\|x^{(k)} - \hat{x}\|$ with Richardson Iteration')
plt.grid(True)
plt.show()

plt.semilogy(K, E)
plt.xlabel('Iteration (k)')
plt.ylabel(r'$\|x^{(k)} - \hat{x}\|$')
plt.title(r'Convergence of $\|x^{(k)} - \hat{x}\|$ with Richardson Iteration')
plt.grid(True)
plt.show()