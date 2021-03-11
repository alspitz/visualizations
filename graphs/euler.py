import argparse

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

from scipy.spatial.transform import Rotation as R

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
parser.add_argument('--plot-both-arcs', action='store_true')
parser.add_argument('order', nargs="?", default="ZYX", choices=["ZYX", "ZXY", "XYZ", "XZY", "YXZ", "YZX"])
args = parser.parse_args()

yaw = np.radians(40)
pitch = -np.radians(50)
roll = -np.radians(30)

fontsize = 18

eulers = args.order
rotation_of_interest = eulers[0]

if eulers == 'ZXY':
  yaw = -np.radians(40)
  roll = np.radians(50)
  pitch = np.radians(30)

axis_angles = [roll, pitch, yaw] # Always in the order: X Y Z
axss = 'XYZ'

Rs = [R.identity()]
euler_inter = np.zeros(3)
for i in range(3):
  euler_inter[i] = axis_angles[axss.index(eulers[i])]
  Rs.append(R.from_euler(eulers, euler_inter))

axes_all = [[rot.apply(ax) for roti, rot in enumerate(Rs)] for ax, axs in zip(np.eye(3), axss)]
axes = [[rot.apply(ax) for roti, rot in enumerate(Rs) if roti != eulers.index(axs)] for ax, axs in zip(np.eye(3), axss)]

style0 = (0, (1, 4, 5, 3))
style1 = (0, (5, 4))
colors = ['red', 'green', 'blue']

linestyles = [style0, style1, 'solid']

styles = [
    dict(linestyle=linestyles[0], linewidth=1),
    dict(linestyle=linestyles[1], linewidth=2),
    dict(linestyle=linestyles[2], linewidth=4)
]

fig = plt.figure(figsize=(16, 12))
ax = plt.axes(projection='3d')

# Plot all axes.
for vs, color in zip(axes, colors):
  for style, v in zip(styles, vs):
    ax.add_artist(Arrow3D([0, v[0]], [0, v[1]], [0, v[2]], arrowstyle='-|>', mutation_scale=25, shrinkA=2, shrinkB=0, joinstyle='miter', zorder=3, color=color, **style))

# Annotate axes.
text_options = dict(fontsize=fontsize, horizontalalignment='center', verticalalignment='center')

for vs, axs in zip(axes, axss):
  ind = eulers.index(axs)
  axlow = axs.lower()
  for i, v in enumerate(vs):
    # Need to increment by one after passing the "no-op" rotation axis
    possi = i if i < ind else i + 1
    s = f"{axlow}_{possi}" if i != ind else f"{axlow}_{i}, {axlow}_{i + 1}"
    ax.text(*(1.08 * v), "$%s$" % s, **text_options)

def plot_arrow_man(point, arrow_dir, normal, length=0.1, head_angle_deg=35, **kwargs):
  d = arrow_dir / np.linalg.norm(arrow_dir)
  n = normal / np.linalg.norm(normal)
  for turn in [head_angle_deg, -head_angle_deg]:
    p = R.from_rotvec(np.radians(turn) * n).apply(-d * length)
    ax.plot([point[0] + p[0], point[0]], [point[1] + p[1], point[1]], [point[2] + p[2], point[2]], **kwargs)

# Plot angle arcs.
N_arc_sample = 51

# Conventional Greek letters
angmap = dict(X="phi", Y="theta", Z="psi")

# Hardcoded for now.
radii = [0.8, 0.6, 0.7]
lengths = [0.04, 0.04, 0.08]

for i in range(3):
  angle = axis_angles[i]
  color = colors[i]

  thetas = np.linspace(0.0, angle, N_arc_sample)

  rotind = eulers.index(axss[i])
  virt_inds = sorted(list(set(range(3)) - set([i])))

  virt_x, virt_y = axes_all[virt_inds[0]][rotind], axes_all[virt_inds[1]][rotind]
  if i == 1:
    thetas = -thetas

  radius = radii[rotind]
  length = lengths[rotind]
  points =  radius * (virt_x[:, None] * np.cos(thetas) + virt_y[:, None] * np.sin(thetas)).T
  points2 = radius * (virt_y[:, None] * np.cos(thetas) - virt_x[:, None] * np.sin(thetas)).T

  # HACK XXX To get symmetry between ZYX and ZXY.
  if eulers == "ZXY" and i in [1, 2] or eulers == "ZYX" and i in [0]:
    points, points2 = points2, points

  ax.plot(points[:, 0], points[:, 1], points[:, 2], color=color, linestyle=linestyles[rotind])

  texts = "\%s" % angmap[axss[i]]
  if angle < 0:
    texts = "-" + texts
  ax.text(*(0.8 * points[len(points) // 2]), "$%s$" % texts, color=color, **text_options)

  # account for negative rotation angles
  actual_rotax = np.cross(axes_all[virt_inds[0]][rotind], axes_all[virt_inds[0]][rotind + 1])

  # Dir is rotation axis x the new axis
  arrowdir =  np.cross(actual_rotax, points[-1, :])
  plot_arrow_man(points[-1, :], arrowdir, normal=axes_all[i][rotind], length=length, color=color, zorder=8)

  if args.plot_both_arcs:
    ax.plot(points2[:, 0], points2[:, 1], points2[:, 2], color=color, linestyle=linestyles[rotind])

    #ax.text(*(0.8 * points2[len(points2) // 2]), "$%s$" % texts, color=color, **text_options)

    arrowdir2 = np.cross(actual_rotax, points2[-1, :])
    plot_arrow_man(points2[-1, :], arrowdir2, normal=axes_all[i][rotind], length=length, color=color, zorder=8)

# Plot x-axis projected onto horizontal plane.
ax_of_int = axes[axss.index(eulers[-1])][-1]
inter_ax = axes[axss.index(eulers[-1])][1]
proj_ax = axes[axss.index(eulers[0])][0]
proj = ax_of_int - ax_of_int.dot(proj_ax) * proj_ax

# Plot square indicating perpendicularity
ax.plot([ax_of_int[0], proj[0]], [ax_of_int[1], proj[1]], [ax_of_int[2], proj[2]], color='black', linestyle='dashed')
sq_len = 0.04
p1 = proj + proj_ax * sq_len
p2 = p1 + inter_ax * sq_len
p3 = p2 - proj_ax * sq_len
ps = [p1, p2, p3]
for i in range(2):
  ax.plot([ps[i][0], ps[i + 1][0]], [ps[i][1], ps[i + 1][1]], [ps[i][2], ps[i + 1][2]], color="black")

ax.set_xlim((-1.05, 1.05))
ax.set_ylim((-1.05, 1.05))
ax.set_zlim((-0.5, 1.0))
ax.set_axis_off()
plt.tight_layout()
#from python_utils.plotu import set_3daxes_equal
#set_3daxes_equal(ax)

# Hand tuned viewpoints
if eulers == 'ZYX':
  ax.elev = 44
  ax.azim = -99
  ax.dist = 6
elif eulers == 'ZXY':
  ax.elev = 44
  ax.azim = -171
  ax.dist = 6

if args.save:
  plt.savefig('euler_%s.pdf' % eulers)

plt.show()
