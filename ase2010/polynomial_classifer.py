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

degrees = range(0,9)
error = []
for d in degrees :
    coef = np.polyfit(x,y,d)

    f_ = np.poly1d(coef)

    f_pred = np.sign(f_(x))
    
    error_rate = np.mean(f_pred != y)
    error.append(error_rate)
    plt.subplot(3, 3, d+1)
    plt.plot(x, f_(x))
    plt.plot(x, f_pred)
    plt.legend(['~f(x)','f^(x)'])
    plt.grid(True)
    plt.title(f'Degree : {d}')

plt.tight_layout()   
plt.show()

plt.plot(degrees, error, color ='blue')
plt.scatter(degrees,error)
plt.title('Evalute error rate')
plt.xlabel('Degrees')
plt.ylabel('Error rate')
plt.grid(True)
plt.show()

print(error)

