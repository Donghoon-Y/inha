import numpy as np
from matplotlib import pyplot as plt

years = [1971, 1972, 1974, 1978, 1982, 1985, 1989, 1993, 1997, 1999, 2000, 2002, 2003]
transistors = [2250, 2500, 5000, 29000, 120000, 275000, 1180000, 3100000, 7500000, 24000000, 42000000, 220000000, 410000000]
log_transistors = []
x = np.zeros(len(years))
A = np.ones((len(years),2))
for i in range(0, len(transistors)) :
    log_transistors.append(np.log10(transistors[i]))

for i in range(0, len(years)) :
    x[i] = years[i]-1970

A[:,1] = np.array(x)
b = np.array(log_transistors)

lstsq_sol = np.linalg.lstsq(A,b)
theta1, theta2 = lstsq_sol[0]

lstsq_sol_log_transistors = theta1 + theta2 * x
lstsq_sol_transistors = 10**lstsq_sol_log_transistors
rms_error = np.sqrt(np.mean((lstsq_sol_log_transistors - log_transistors)**2))

plt.figure(figsize=(10, 6))
plt.scatter(years, transistors, color = 'green')
plt.plot(years, lstsq_sol_transistors, color = 'red')
plt.yscale('log') 
plt.xlabel('Years')
plt.ylabel('Transistors')
plt.legend(['Data', 'Lstsq Solution'])
plt.grid(True)
plt.show()


