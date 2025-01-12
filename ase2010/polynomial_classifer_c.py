import numpy as np
from matplotlib import pyplot as plt

n = 200 
x = np.linspace(-1, 1, n)
y = np.random.uniform(-1,1,n)
for i in range(len(x)) :
    if -0.5<= x[i] < 0.1 or 0.5 <= x[i] :
        y[i] = 1

    else :
        y[i] = -1

degrees = 3

coef = np.polyfit(x,y,degrees)

f_ = np.poly1d(coef)

f_pred = np.sign(f_(x))
f_c = lambda x : 3*(x+0.5)*(x-0.1)*(x-0.5)
    
error_rate = np.mean(f_pred != y)

plt.plot(x,f_(x), color = 'blue', label = 'Polynomial')
plt.plot(x,f_c(x),color = 'red', label = 'Given')
plt.grid(True)
plt.legend()
plt.show()



    

