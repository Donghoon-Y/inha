import numpy as np
from matplotlib import pyplot as plt

n = 200

x = np.random.uniform(-1,1, size=(n,2))
y = np.random.uniform(-1,1,n)

for i in range(len(x)) :
    if x[i][0]*x[i][1] >= 0 :
        y[i] = 1

    else :
        y[i] = -1


X_poly = np.column_stack([np.ones(n), x[:, 0], x[:, 1], 
                          x[:, 0]**2, x[:, 0] * x[:, 1], x[:, 1]**2])
coef = np.linalg.lstsq(X_poly,y)[0]
y_pred = np.sign(X_poly@coef)
error_rate = np.mean(y_pred != y)


print(coef)


x1_range = np.linspace(-1, 1, 400)
x2_range = np.linspace(-1, 1, 400)
x1_grid, x2_grid = np.meshgrid(x1_range, x2_range)
X_grid_poly = np.column_stack([
    np.ones(x1_grid.size), x1_grid.ravel(), x2_grid.ravel(),
    x1_grid.ravel() ** 2, x1_grid.ravel() * x2_grid.ravel(), x2_grid.ravel() ** 2
])

f_values = X_grid_poly @ coef
f_values = f_values.reshape(x1_grid.shape)
misclassified = n*error_rate

print(f'{misclassified}')

plt.figure(figsize=(8, 6))
plt.scatter(x[y == 1, 0], x[y == 1, 1], color='green', label="y = +1")
plt.scatter(x[y == -1, 0], x[y == -1, 1], color='red', label="y = -1")
plt.contour(x1_grid, x2_grid, f_values, levels=[0], colors='black')
plt.axhline(0, color='gray', linewidth=0.5, linestyle='--')
plt.axvline(0, color='gray', linewidth=0.5, linestyle='--')
plt.legend()
plt.title("Polynomial Classifier Decision Boundary")
plt.xlabel("x1")
plt.ylabel("x2")
plt.show()