import yaml
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

with open('/home/neo/workspace/logs/voronoi_planner.yaml', 'r') as file:
  data = yaml.safe_load(file)

# plot
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

colors = plt.cm.viridis_r(np.linspace(0, 1, 8))  # plasma_r, inferno_r, magma_r, viridis_r, cividis_r

scale_z = 1

vertices = np.array(data['vertices'])
ridges = data['ridges']
altitudes = data['altitudes']
v_lengths = data['v_lengths']
r_lengths = data['r_lengths']
r_lengths[-1] = len(ridges)
start = data['start']
goal = data['goal']

# ridges
for i in range(len(altitudes)+1):
  for simplex in ridges[r_lengths[i]:r_lengths[i+1]]:
    p1 = vertices[simplex[0]]
    p2 = vertices[simplex[1]]
    x = [p1[0], p2[0]]
    y = [p1[1], p2[1]]
    z = [p1[2]*scale_z, p2[2]*scale_z]

    idx = int(z[0]/scale_z * 5 - 7)
    ax.plot(x, y, z, color=colors[idx], marker='', linestyle='-', markersize=5)

# vertices
for i in range(len(altitudes)):
  x = vertices[v_lengths[i]:v_lengths[i+1], 0]
  y = vertices[v_lengths[i]:v_lengths[i+1], 1]
  z = np.array(vertices[v_lengths[i]:v_lengths[i+1], 2])*scale_z

  idx = int(z[0]/scale_z * 5 - 7)
  ax.plot(x, y, z, color=colors[idx], marker='o', linestyle='', markersize=5)
# for i in range(len(vertices)):
#   x = vertices[i, 0]
#   y = vertices[i, 1]
#   z = vertices[i, 2]*scale_z
#   ax.text(x, y, z, f'{i}')

# start and goal
ax.plot(start[0], start[1], start[2], color='r', marker='o', linestyle='', markersize=5)
ax.plot(goal[0], goal[1], goal[2], color='r', marker='o', linestyle='', markersize=5)

# Imposta i limiti degli assi
ax.set_xlim(0, 20.0)
ax.set_ylim(0, 10.0)
ax.set_zlim(0, 3.0*scale_z)

# Etichette degli assi
#ax.set_aspect('equal')

# Mostra il grafico
plt.show()