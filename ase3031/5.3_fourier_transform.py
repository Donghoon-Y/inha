import numpy as np
from matplotlib import pyplot as plt

w = np.linspace(-50, 50, 1000)

f1 = lambda w : np.sqrt(np.pi)*np.exp(-w**2/4)
f2 = lambda w : np.sqrt(np.pi)/2*(np.exp(-(w-5)**2/4)+np.exp(-(w+5)**2/4))
f = lambda w : np.sqrt(np.pi)*np.exp(-w**2/4) + np.sqrt(np.pi)/2*(np.exp(-(w-5)**2/4)+np.exp(-(w+5)**2/4))

plt.plot(w, f1(w), label='f1')
plt.plot(w, f2(w), label='f2')
plt.plot(w, f(w), label = 'f')
plt.xlabel('w')
plt.ylabel('F(w)')
plt.title('fourier Transform')
plt.legend()
plt.grid(True)
plt.show()