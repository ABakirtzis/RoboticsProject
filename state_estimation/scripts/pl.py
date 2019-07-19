from __future__ import division
from matplotlib import pyplot as plt

x, y = [],[]
fhandle = open("veldata.txt", 'r')
for i in fhandle:
    temp = i.strip().split(",")
    x.append(float(temp[0]))
    y.append(float(temp[1]))

x1 = [y[i] for i in range(len(x)) if x[i] > 0.1]
x2 = [y[i] for i in range(len(x)) if x[i] < 0.1]

plt.plot(x, label = "desired v")
plt.plot(y, label = "odometry v")
plt.xlabel("t(s)")
plt.ylabel("v(m/s)")
plt.title("Real vs Desired Velocity")
plt.legend()
plt.show()
mean1 = sum(x1) / len(x1)
mean2 = sum(x2) / len(x2)
print "mean for cmdvel = 0.2: {}".format(mean1)
print "mean for cmdvel = 0: {}".format(mean2)
print "std for cmdvel = 0.2: {}".format((sum([(i - mean1) ** 2 for i in x1])/(len(x1) - 1)) ** (1/2))
print "std for cmdvel = 0: {}".format((sum([(i - mean2) ** 2 for i in x2])/(len(x2) - 1)) ** (1/2))
