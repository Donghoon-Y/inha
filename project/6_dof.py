import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# Constants
earth_radius = 6371  # Radius of Earth in kilometers
satellite_altitude = 500  # Altitude of the satellite in kilometers
satellite_semi_major_axis = earth_radius + satellite_altitude  # Total radius of the satellite's orbit
mu = 398600  # [km^3/s^2] Earth gravitational constant
e = 0.01  # Eccentricity
p = satellite_semi_major_axis * (1 - e**2)  # [km] Semi-latus rectum
h = np.sqrt(mu*p)
n = np.sqrt(mu/satellite_semi_major_axis**3)
T = 2*np.pi/n

tspan = (0,T)
teval = np.linspace(0, T, 100)

# True anomaly range
f = np.linspace(0, 2 * np.pi, 500)  # [rad] True anomaly over one orbital period

# Calculate satellite trajectory
r = p / (1 + e * np.cos(f))
dt_dr = mu/h*e*np.sin(f)  # Orbital radius as a function of true anomaly
dt_dtheta = h/r**2
rfdot = mu/h*(1+e*np.cos(f))
x_orbit = r * np.cos(f)  # X-coordinates of orbit
y_orbit = r * np.sin(f)  # Y-coordinates of orbit
z_orbit = np.zeros_like(f)  # Z-coordinates fixed for equatorial orbit

x0 = r[0]*np.cos(f[0])
y0 = r[0]*np.sin(f[0])
z0 = 0

vx0 = dt_dr[0]*np.cos(f[0]) - rfdot[0]*np.sin(f[0])
vy0 = dt_dr[0]*np.sin(f[0]) + rfdot[0]*np.cos(f[0])
vz0 = 0
r0 = [x0,y0,z0]
v0 = [vx0,vy0,vz0]

initial_state = np.hstack([r0,v0])

def equations(t,y) :
    r = y[0:3]
    v = y[3:]
    rmag = np.linalg.norm(r)

    rdot = v
    vdot = - mu / (rmag**3) * r
    return np.hstack([rdot, vdot])

sol = solve_ivp(equations, tspan, initial_state, t_eval =teval)

x, y, z = sol.y[:3, :]

# Visualization
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

# Plot Earth's surface
u = np.linspace(0, 2 * np.pi, 100)
v = np.linspace(0, np.pi, 100)
earth_x = earth_radius * np.outer(np.cos(u), np.sin(v))
earth_y = earth_radius * np.outer(np.sin(u), np.sin(v))
earth_z = earth_radius * np.outer(np.ones(np.size(u)), np.cos(v))

ax.plot_surface(earth_x, earth_y, earth_z, color='blue', alpha=0.5)

# Plot precomputed analytical trajectory
ax.plot(x_orbit, y_orbit, z_orbit, color='green', label='Analytical Orbit (Precomputed)', linewidth=2)

# Plot numerical integration results
ax.scatter(x, y, z, color='red', label='Numerical Integration', s=10)

# Configure plot
ax.set_title("3D Satellite Orbit Visualization", fontsize=16)
ax.set_xlabel("X (km)", fontsize=12)
ax.set_ylabel("Y (km)", fontsize=12)
ax.set_zlabel("Z (km)", fontsize=12)
ax.legend()

plt.show()
