import numpy as np

def mtx_mult(A,B) :
  np.array(A)
  np.array(B)
  result = np.dot(A, B)
  return result

a = [[1,2,3],[2,1,3],[2,1,2]]
b = [[1,0,0],[0,1,0],[0,0,1]]
mtx_mult(a,b)

phi = 10
theta = 20
psi = -60
##### Don't modify this cell #####

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Creating DCM

D2R = np.pi/180
R_phi = [[1, 0, 0], [0, np.cos(phi*D2R), np.sin(phi*D2R)], [0, -np.sin(phi*D2R), np.cos(phi*D2R)]]
R_theta = [[np.cos(theta*D2R), 0, -np.sin(theta*D2R)], [0, 1, 0], [np.sin(theta*D2R), 0, np.cos(theta*D2R)]]
R_psi = [[np.cos(psi*D2R), np.sin(psi*D2R), 0], [-np.sin(psi*D2R), np.cos(psi*D2R), 0], [0, 0, 1]]

NED2Euler = np.array(mtx_mult(mtx_mult(R_phi, R_theta), R_psi))
Euler2NED = NED2Euler.T

# Visualization

a = 1.5
b = 1
c = 0.3
d = 2

Euler_1 = [a*2/3, 0, 0]
Euler_2 = [-a/3, b/2, 0]
Euler_3 = [-a/3, -b/2, 0]
Euler_4 = [-a/3, 0, 0]
Euler_5 = [-a/3, 0, -c]

ax_Euler_1 = [d, 0, 0]
ax_Euler_2 = [0, d, 0]
ax_Euler_3 = [0, 0, d]

NED_1 = Euler2NED@Euler_1
NED_2 = Euler2NED@Euler_2
NED_3 = Euler2NED@Euler_3
NED_4 = Euler2NED@Euler_4
NED_5 = Euler2NED@Euler_5

ax_NED_1 = Euler2NED@ax_Euler_1
ax_NED_2 = Euler2NED@ax_Euler_2
ax_NED_3 = Euler2NED@ax_Euler_3

fig = plt.figure(figsize=(10, 10))
plot_plane = fig.add_subplot(111,projection='3d')
plot_plane.plot([0, 0], [0, d], [0, 0], 'r:', label = 'N')
plot_plane.plot([0, d], [0, 0], [0, 0], 'g:', label = 'E')
plot_plane.plot([0, 0], [0, 0], [0, -d], 'b:', label = 'D')
plot_plane.scatter(0, -d, 0, alpha = 0)
plot_plane.scatter(-d, 0, 0, alpha = 0)
plot_plane.scatter(0, 0, d, alpha = 0)
plot_plane.plot([NED_1[1], NED_2[1]], [NED_1[0], NED_2[0]], [-NED_1[2], -NED_2[2]], 'k')
plot_plane.plot([NED_1[1], NED_3[1]], [NED_1[0], NED_3[0]], [-NED_1[2], -NED_3[2]], 'k')
plot_plane.plot([NED_1[1], NED_4[1]], [NED_1[0], NED_4[0]], [-NED_1[2], -NED_4[2]], 'k')
plot_plane.plot([NED_1[1], NED_5[1]], [NED_1[0], NED_5[0]], [-NED_1[2], -NED_5[2]], 'k')
plot_plane.plot([NED_2[1], NED_3[1]], [NED_2[0], NED_3[0]], [-NED_2[2], -NED_3[2]], 'k')
plot_plane.plot([NED_4[1], NED_5[1]], [NED_4[0], NED_5[0]], [-NED_4[2], -NED_5[2]], 'k')
plot_plane.plot([0, ax_NED_1[1]], [0, ax_NED_1[0]], [0, -ax_NED_1[2]], 'r', label = 'x')
plot_plane.plot([0, ax_NED_2[1]], [0, ax_NED_2[0]], [0, -ax_NED_2[2]], 'g', label = 'y')
plot_plane.plot([0, ax_NED_3[1]], [0, ax_NED_3[0]], [0, -ax_NED_3[2]], 'b', label = 'z')
plot_plane.set_xlabel('East')
plot_plane.set_ylabel('North')
plot_plane.set_zlabel('Up')
plot_plane.legend()
plt.show()