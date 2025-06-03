import numpy as np
from matplotlib import pyplot as plt
import math 
####################### 1.2 ####################
n = 10 
Px = []
Py = []
for i in range(0,n+1) :
    px = math.comb(n,i)*(0.3)**i*(0.7)**(n-i)
    py = math.comb(n,i)*(0.7)**i*(0.3)**(n-i)
    Px.append(px)
    Py.append(py)
fig, ax1 = plt.subplots()

ax1.plot(range(0,n+1), Px, color = 'red')
ax1.set_ylabel('Px(x)', color = 'r')
ax1.set_xlabel('x, y')
ax1.tick_params(axis='y', labelcolor='red')
ax1.grid(True)

ax2 = ax1.twinx()
ax2.plot(range(0,n+1), Py, color = 'blue')
ax2.set_ylabel('Py(y)', color = 'b')
ax2.tick_params(axis='y', labelcolor='blue')

plt.show()

####################### 1.3 ####################
trials = np.random.choice(range(0,n+1), 10000, p = Px)

fig, ax3 = plt.subplots()
ax3.hist(trials, color = 'b')
ax3.set_xlabel('x')
ax3.set_ylabel('number of samples', color = 'b')
ax3.tick_params(axis='y', labelcolor='blue')
ax3.grid(True)

ax4 = ax3.twinx()
ax4.plot(range(0,n+1), Px,color= 'r')
ax4.set_ylabel('Px(x)', color = 'r')
ax4.tick_params(axis='y', labelcolor='red')
plt.show()