from __future__ import division
from matplotlib import pyplot as plt
import numpy as np

titloi = ['Position', 'Position Error', 'X', 'Y', 'Velocity', 'Velocity Error', 'Angle', 'Angle Error']
xlabels = ['x(m)', 't(s)', 't(s)', 't(s)', 't(s)', 't(s)', 't(s)', 't(s)']
ylabels = ['y(m)', 'd(m)', 'x(m)', 'y(m)', 'v(m/s)', 'verr(m/s)', 'theta(rad)', 'dtheta(rad)']
titlos = 0

for f in ['xy', 'xyreal']:
	z = [tuple(map(float, i.split(';'))) for i in open(f, 'r').read().split('\n')[:-1] if i != '']
	x = [i[0] for i in z]
	y = [i[1] for i in z]
        if f == 'xy':
                x1 = x
                y1 = y
        else:
                x2 = x
                y2 = y
	plt.plot(x,y, label = 'Estimation' * (f == 'xy') + 'Odometry' * (f == 'xyreal'))
plt.legend()
plt.xlabel(xlabels[titlos])
plt.ylabel(ylabels[titlos])
plt.title(titloi[titlos])
titlos+=1
plt.show()
plt.plot([(i1 - i2) ** 2 + (j1 - j2) ** 2 for (i1,j1), (i2,j2) in zip(zip(x1,y1), zip(x2,y2))])
print ("position mean error: {}".format(sum([(i1 - i2) ** 2 + (j1 - j2) ** 2 for (i1,j1), (i2,j2) in zip(zip(x1,y1), zip(x2,y2))])/len(x1)))
plt.xlabel(xlabels[titlos])
plt.ylabel(ylabels[titlos])
plt.title(titloi[titlos])
titlos+=1
plt.show()
plt.plot(x1, label = 'Estimation')
plt.plot(x2, label = 'Odometry')
plt.xlabel(xlabels[titlos])
plt.ylabel(ylabels[titlos])
plt.title(titloi[titlos])
plt.legend()
titlos+=1
plt.show()
plt.plot(y1, label = 'Estimation')
plt.plot(y2, label = 'Odometry')
plt.legend()
plt.xlabel(xlabels[titlos])
plt.ylabel(ylabels[titlos])
plt.title(titloi[titlos])
titlos+=1
plt.show()
for f in ["velocity", "angle"]:
        z = [tuple(map(float, i.split(';'))) for i in open(f, 'r').read().split('\n') if i != '']
        plt.plot([i for i,j in z], label = 'Estimation')
        plt.plot([j for i,j in z], label = 'Odometry')
        plt.legend()
        plt.xlabel(xlabels[titlos])
        plt.ylabel(ylabels[titlos])
        plt.title(titloi[titlos])
        titlos+=1
        plt.show()
        plt.plot([i-j for i,j in z])
        plt.xlabel(xlabels[titlos])
        plt.ylabel(ylabels[titlos])
        plt.title(titloi[titlos])
        print ("{} mean error: {}".format(titloi[titlos], sum([abs(i-j) for i,j in z]) / len(z)))
        titlos+=1
        plt.show()


