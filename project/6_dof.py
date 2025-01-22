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

# Create figure
fig = plt.figure(figsize=(14, 9))
ax = fig.add_subplot(111, projection='3d')

# Generate Earth's surface
u = np.linspace(0, 2 * np.pi, 100)
v = np.linspace(0, np.pi, 100)
x = earth_radius * np.outer(np.cos(u), np.sin(v))
y = earth_radius * np.outer(np.sin(u), np.sin(v))
z = earth_radius * np.outer(np.ones(np.size(u)), np.cos(v))

# Plot Earth's surface
ax.plot_surface(x, y, z, color='blue', alpha=0.7)

# True anomaly range
f = np.linspace(0, 2 * np.pi, 500)  # [rad] True anomaly over one orbital period

# Calculate satellite trajectory
r = p / (1 + e * np.cos(f))  # Orbital radius as a function of true anomaly
x_orbit = r * np.cos(f)  # X-coordinates of orbit
y_orbit = r * np.sin(f)  # Y-coordinates of orbit
z_orbit = np.zeros_like(f)  # Z-coordinates fixed for equatorial orbit

# Plot satellite trajectory
ax.plot(x_orbit, y_orbit, z_orbit, color='red', label='Satellite Orbit')

# Annotate and configure the plot
ax.set_title("Satellite Orbit Around Earth", fontsize=16)
ax.set_xlabel("X (km)", fontsize=12)
ax.set_ylabel("Y (km)", fontsize=12)
ax.set_zlabel("Z (km)", fontsize=12)
ax.legend()
ax.set_box_aspect([1, 1, 1])  # Equal aspect ratio

# Display the plot
plt.show()
