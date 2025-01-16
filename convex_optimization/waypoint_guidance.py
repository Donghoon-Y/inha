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

print(S)

