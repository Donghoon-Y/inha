import numpy as np
from matplotlib import pyplot as plt

N = [3, 5, 10, 30]
a0 = 3*np.pi/8
an = lambda n : (np.pi*n*np.sin(np.pi*n/2) - 2*np.cos(n*np.pi/2) + 2)/(np.pi*n**2)
bn = 0 
x = np.linspace(-np.pi, np.pi, 1000)
# f(x) 정의
f_x = np.piecewise(x,
                   [x < -np.pi/2,
                    (x >= -np.pi/2) & (x < 0),
                    (x >= 0) & (x < np.pi/2),
                    x >= np.pi/2],
                   [0,
                    lambda x: np.pi + x,
                    lambda x: np.pi - x,
                    0])

for i in N :
    Sn = a0
    for j in range(1,i+1) :
        Sn += an(j) * np.cos(j * x)
    plt.plot(x, Sn, label = f'N={j}')


plt.plot(x,f_x, 'r--', label = 'f(x)')
plt.xlabel('X')
plt.ylabel('f(x)')
plt.grid(True)
plt.legend()
plt.show()
