from __future__ import division
import numpy as np
from matplotlib import pyplot as plt
import curvemath
import linemath

#planning
start = (0.30, 0.45)
end = (0.2, -0.45)
startang_robot = -np.pi/2
obstacles = [((0.30, 0.15), (-0.30, -0.15))]
resolution = 80
size = 1.5
safety_net = 0.13

cur,obs,obs_s = curvemath.find_curve(start = start, end = end, resolution = resolution, safety_net = safety_net, obstacles = obstacles, smoothing_range = 0.8, plot_ret = True)
startang_curve = np.arctan2(cur[1][1] - cur[0][1], cur[1][0] - cur[0][0])
startangle = linemath.norm(startang_curve - startang_robot)
with open("info", 'r') as fhandle2:
    z = fhandle2.read().split('\n')[:-1]
    x1 = [float(i.split(' ')[0].split(';')[0]) for i in z]
    y1 = [float(i.split(' ')[0].split(';')[1]) for i in z]
    
plt.plot([i[0] for i in obs], [i[1] for i in obs], 'o', label = 'obstacle')
plt.plot([i[0] for i in obs_s], [i[1] for i in obs_s], 'o', label = 'safety_net')
plt.plot([i[0] for i in cur], [i[1] for i in cur], 'o', label = 'curve to follow')
plt.plot(x1, y1, '--', linewidth = 2, label = 'robot position')
plt.title("Experiment")
plt.legend()
plt.xlabel('x(m)')
plt.ylabel('y(m)')
plt.xlim(-0.75, 0.75)
plt.ylim(-0.75, 0.75)
plt.show()
