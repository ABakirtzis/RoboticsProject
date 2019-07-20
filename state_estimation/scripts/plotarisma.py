from __future__ import division
from matplotlib import pyplot as plt
import numpy as np

titloi = ['Position', 'Position Error', 'X', 'Y', 'Velocity', 'Velocity Error', 'Angle', 'Angle Error']
xlabels = ['x(m)', 't(s)', 't(s)', 't(s)', 't(s)', 't(s)', 't(s)', 't(s)']
ylabels = ['y(m)', 'd(m)', 'x(m)', 'y(m)', 'v(m/s)', 'verr(m/s)', 'theta(rad)', 'dtheta(rad)']
titlos = 0

for f in ['xy']:
	z = [tuple(map(float, i.split(';'))) for i in open(f, 'r').read().split('\n')[:-1] if i != '']
	x = [i[0] for i in z]
	y = [i[1] for i in z]
        if f == 'xy':
                x1 = x
                y1 = y
        else:
                x2 = x
                y2 = y
	plt.plot(x,y)
#plt.xlim(-0.75, 0.75)
#plt.ylim(-0.75, 0.75)
plt.xlabel(xlabels[titlos])
plt.ylabel(ylabels[titlos])
plt.title(titloi[titlos])
titlos+=1
plt.show()
