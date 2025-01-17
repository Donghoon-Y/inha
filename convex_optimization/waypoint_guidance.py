import numpy as np
import matplotlib.pyplot as plt
import scipy.sparse as ssp
import scipy.sparse.linalg as sla

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

u_hat = sla.lsqr(S@H, -S@G@x_0+wp.T.flatten())[0] #transpose하고 flatten해야 x1,y1,x2,y2 이런 순서의 벡터로 쌓인다.

u_opt = u_hat.reshape((1000,2)).T

x = np.zeros((4,n+1))
x[:,0] = x_0

for i in range(n) :
   x[:, i+1] = A@(x[:,i]) + B@(u_opt[:,i])

plt.figure(figsize=(14,9), dpi = 100)
plt.subplot(2,2,1)
plt.plot(ts, x[0,:], label = 'x position')
plt.xlabel('T')
plt.ylabel('x position')
plt.legend()
plt.grid(True)

plt.subplot(2,2,2)
plt.plot(ts, x[1,:], label = 'y position')
plt.xlabel('T')
plt.ylabel('y position')
plt.legend()
plt.grid(True)

plt.subplot(2,2,3)
plt.plot(ts, x[2,:], label = 'x-dir velocity')
plt.xlabel('T')
plt.ylabel('x-dir velocity')
plt.legend()
plt.grid(True)

plt.subplot(2,2,4)
plt.plot(ts, x[3,:], label = 'y-dir velocity')
plt.xlabel('T')
plt.ylabel('y-dir velocity')
plt.legend()
plt.grid(True)
plt.show()

plt.figure(figsize=(14,9), dpi=100)
plt.subplot(2,2,1)
plt.plot(ts[:-1], u_opt[0,:])
plt.xlabel('T')
plt.ylabel('r$u_1$')
plt.grid(True)

plt.subplot(2,2,2)
plt.plot(ts[:-1], u_opt[1,:])
plt.xlabel('T')
plt.ylabel('r$u_2$')
plt.grid(True)
plt.show()

plt.figure(figsize=(14,9), dpi=100)
plt.plot(x_0[0], x_0[1], 'o', markersize=10, label='Initial position')
for k in range(K):
  plt.plot(wp[0,k], wp[1,k], '^', markersize=10, label=f'Waypoint #{k+1}')
plt.title('Trajectory')
plt.plot(x[0,:],x[1,:], label='Optimal trajectory')
plt.legend()
for i in range(0,n-1,10):
  plt.arrow(x[0,i], x[1,i], 2*u_opt[0,i], 2*u_opt[1,i], head_width=1, width=0.2, fc='tab:red', ec='none')
plt.axis('equal')
plt.xlabel(r'$x$ position')
plt.ylabel(r'$y$ position')
plt.grid()
plt.show()