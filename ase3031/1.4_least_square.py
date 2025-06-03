import numpy as np
from matplotlib import pyplot as plt

x = np.array([0, 1, 2, 3])
y = np.array([1, 2, 9, 4])
least_squares_alpha = 0
least_squares_beta = 4

weighted_least_squares_alpha = 0.98532
weighted_least_squares_beta = 1.044

function_least_squares = lambda x: least_squares_alpha + least_squares_beta * x
function_weighted_least_squares = lambda x: weighted_least_squares_alpha + weighted_least_squares_beta * x


plt.scatter(x,y, label='Data Points')
plt.plot(x, function_least_squares(x), label='Least Squares Fit', color='blue')
plt.plot(x, function_weighted_least_squares(x), label='Weighted Least Squares Fit', color='orange')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Least Squares vs Weighted Least Squares')
plt.grid(True)
plt.legend()
plt.show()