import argparse

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

from scipy.spatial.transform import Rotation as R

from python_utils.mathu import e1, e2, e3
from python_utils.plotu import set_3daxes_equal

# Enable latex compatible font.
matplotlib.rcParams['mathtext.fontset'] = 'cm'

class Arrow3D(FancyArrowPatch):
  def __init__(self, xs, ys, zs, *args, **kwargs):
    FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
    self._verts3d = xs, ys, zs

  def draw(self, renderer):
    xs3d, ys3d, zs3d = self._verts3d
    xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
    self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
    FancyArrowPatch.draw(self, renderer)

parser = argparse.ArgumentParser()
parser.add_argument('--save', action='store_true')
args = parser.parse_args()

yaw = np.radians(40) # Yaw
pitch = np.radians(50) # Pitch
roll = np.radians(30) # Roll

fontsize = 18

eulers = 'ZYX'

o = np.zeros(3)
R1 = R.from_euler(eulers, [yaw, 0, 0])
R2 = R.from_euler(eulers, [yaw, -pitch, 0])
R3 = R.from_euler(eulers, [yaw, -pitch, -roll])

fig = plt.figure(figsize=(16, 12))
ax = plt.axes(projection='3d')

x0 = e1
x1 = R1.apply(e1)
x2 = x3 = R3.apply(e1)
assert np.allclose(R2.apply(e1), R3.apply(e1))

y0 = e2
y1 = y2 = R2.apply(e2)
y3 = R3.apply(e2)
assert np.allclose(R1.apply(e2), R2.apply(e2))

z0 = z1 = e3
z2 = R2.apply(e3)
z3 = R3.apply(e3)
assert np.allclose(e3, R1.apply(e3))

style0 = (0, (1, 4, 5, 3))
style1 = (0, (5, 4))
colors = ['red', 'green', 'blue']

styles = [
    dict(linestyle=style0, linewidth=1),
    dict(linestyle=style1, linewidth=2),
    dict(linestyle='solid', linewidth=4)
]

psets = [
  [x0, y0, z1],
  [x1, y2, z2],
  [x3, y3, z3]
]

# Plot all axes.
for style, pset in zip(styles, psets):
  for v, color in zip(pset, colors):
    #if color == 'green': continue
    ax.add_artist(Arrow3D([o[0], v[0]], [o[1], v[1]], [o[2], v[2]], arrowstyle='-|>', mutation_scale=25, shrinkA=2, shrinkB=0, joinstyle='miter', zorder=3, color=color, **style))

# Annotate axes.
text_options = dict(fontsize=fontsize, horizontalalignment='center', verticalalignment='center')
ax.text(*(1.05 * x0), "$x_0$", **text_options)
ax.text(*(1.05 * x1), "$x_1$", **text_options)
ax.text(*(1.05 * x3), "$x_2, x_3$", **text_options)

ax.text(*(1.05 * y0), "$y_0$", **text_options)
ax.text(*(1.10 * y2), "$y_1, y_2$", **text_options)
ax.text(*(1.10 * y3), "$y_3$", **text_options)

ax.text(*(1.05 * z1), "$z_0, z_1$", **text_options)
ax.text(*(1.10 * z2), "$z_2$", **text_options)
ax.text(*(1.05 * z3), "$z_3$", **text_options)

def plot_arrow_man(point, arrow_dir, normal, length=0.1, head_angle_deg=35, **kwargs):
  d = arrow_dir / np.linalg.norm(arrow_dir)
  n = normal / np.linalg.norm(normal)
  for turn in [head_angle_deg, -head_angle_deg]:
    p = R.from_rotvec(np.radians(turn) * n).apply(-d * length)
    ax.plot([point[0] + p[0], point[0]], [point[1] + p[1], point[1]], [point[2] + p[2], point[2]], **kwargs)

# Plot angle arcs.
N_arc_sample = 51

radius = 0.8
thetas = np.linspace(0.0, yaw, N_arc_sample)
points = radius * (x0[:, None] * np.cos(thetas) + y0[:, None] * np.sin(thetas)).T
ax.plot(points[:, 0], points[:, 1], points[:, 2], color='blue', **styles[0])
tp = 0.8 * points[len(points) // 2]
ax.text(*tp, "$\psi$", color='blue', **text_options)
plot_arrow_man(radius * x1, np.cross(z1, x1), normal=z0, length=0.04, color='blue', zorder=8)

radius = 0.6
thetas = np.linspace(0.0, pitch, N_arc_sample)
points = radius * (x1[:, None] * np.cos(thetas) + z1[:, None] * np.sin(thetas)).T
ax.plot(points[:, 0], points[:, 1], points[:, 2], color='green', linestyle=style1)
tp = 0.8 * points[len(points) // 2]
ax.text(*tp, "$-\\theta$", color='green', **text_options)
plot_arrow_man(radius * x2, np.cross(x2, y2), normal=y1, length=0.04, color='green', zorder=8)

radius = 0.7
thetas = np.linspace(0.0, roll, N_arc_sample)
points = radius * (z2[:, None] * np.cos(thetas) + y2[:, None] * np.sin(thetas)).T
ax.plot(points[:, 0], points[:, 1], points[:, 2], color='red', linestyle='solid')
tp = 0.8 * points[len(points) // 2]
ax.text(*tp, "$-\phi$", color='red', **text_options)
plot_arrow_man(radius * z3, y3, normal=x2, length=0.08, color='red', zorder=8)

# Plot x-axis projected onto horizontal plane.
ax.plot([x3[0], x3[0]], [x3[1], x3[1]], [x3[2], x1[2]], color='black', linestyle='dashed')

ax.set_xlim((-1.0, 1.0))
ax.set_ylim((-1.0, 1.0))
ax.set_zlim((-0.5, 1.0))

ax.set_axis_off()

plt.tight_layout()
#set_3daxes_equal(ax)
ax.elev = 44
ax.azim = -99
ax.dist = 6

if args.save:
  plt.savefig('out.pdf')

plt.show()
