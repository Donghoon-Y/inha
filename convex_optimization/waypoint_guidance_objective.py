#waypoint를 정확하게 지나지 않아도 되기 때문에 어느정도 오차를 허용한다고 생각하여 multi objective least sqaure 문제로 생각하고 문제를 해결한다.
import numpy as np
import matplotlib.pyplot as plt
import scipy.sparse.linalg as sla
import scipy.sparse as ssp

n = 1000
T = 50
ts = np.linspace(0, T, n+1)
delta = T/n
gamma = 0.05

A = np.array([[1, 0, (1-0.5*gamma*delta)*delta, 0],
             [0, 1, 0, (1-0.5*gamma*delta)*delta],
             [0, 0, 1-gamma*delta, 0],
             [0, 0, 0, 1-gamma*delta]])
B = np.array([[0.5*delta**2, 0],
             [0, 0.5*delta**2],
             [delta, 0],
             [0, delta]])

C = np.array([[1,0,0,0],
             [0,1,0,0]])

x_0 = np.array([10, -20, 30, -10])

K = 3
tk = np.array([ 300, 600, 1000])-1
wp = np.array([[100,  50,  100],
               [  0,  50,   50]])

G  = np.zeros((2*n, 4))
for i in range(n) :
    G[2*i:2*i+2,:] = C@np.linalg.matrix_power(A, i+1)

H = np.zeros((2*n, 2*n))
H_first = np.zeros((2*n, 2))
for i in range(n) :
    H_first[2*i:2*(i+1), :] = C@np.linalg.matrix_power(A,i)@B

for i in range(n):
  H[2*i:,2*i:2*(i+1)] = H_first[:2*(n-i),:] #column으로 넣는 듯

S = np.zeros((2*K, 2*n))
for i in range(K) :
   S[2*i:2*(i+1), 2*tk[i]:2*tk[i]+2] = np.eye(2)

W = ssp.diags([1, 1, 1, 1, 10, 10])

A_tilde = np.vstack((np.eye(2*n,2*n), W@S@H))
y_tilde = np.hstack((np.zeros(2*n), W@(wp.T.flatten() - S@G@x_0)))

u_hat = sla.lsqr(A_tilde, y_tilde)[0]

u_opt = u_hat.reshape((1000,2)).T

x = np.zeros((4,n+1))
x[:,0] = x_0
for i in range(n) :
   x[:,i+1] = A@(x[:,i]) + B@(u_opt[:,i])

plt.figure(figsize=(14,9), dpi=100)
plt.plot(x_0[0],x_0[1], marker = 'o', markersize = 10, label = 'Initial Position')
for k in range(K) :
   plt.plot(wp[0,k],wp[1,k], marker = '^', markersize =10, label=f'Waypoint{k+1}')
plt.plot(x[0,:],x[1,:], label = 'Optimal trajectory')
plt.xlabel('x postition')
plt.ylabel('y position')
plt.grid()
plt.legend()
plt.show()