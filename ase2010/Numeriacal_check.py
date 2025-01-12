import numpy as np
from matplotlib import pyplot as plt


m, n = 30, 10

A = np.random.randn(m,n)
b = np.random.randn(m)
d1 = np.random.randn(n)
d2 = np.random.randn(n)
d3 = np.random.randn(n)

x_hat = np.linalg.pinv(A)@b

optimal_error = np.linalg.norm(A@x_hat-b)**2
error1 = np.linalg.norm(A@(x_hat+d1)-b)**2
error2 = np.linalg.norm(A@(x_hat+d2)-b)**2
error3 = np.linalg.norm(A@(x_hat+d3)-b)**2

plt.scatter(1,optimal_error, color = 'black', label = 'least sqaues sol')
plt.scatter(1,error1, color = 'g', label = 'common sol1')
plt.scatter(1,error2, color = 'r', label = 'common sol2')
plt.scatter(1,error3, color = 'b', label = 'common sol3')
plt.grid(True)
plt.legend()
plt.show()

print(f'{optimal_error},{error1},{error2},{error3}')
