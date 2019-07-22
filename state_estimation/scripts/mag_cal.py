from matplotlib import pyplot as plt
import numpy as np


with open("xy_cal", 'r') as fhandle:
    z = [i.split(';') for i in fhandle.read().split('\n')[:-1]]
x = np.array([float(i) for i,j in z])
y = np.array([float(j) for i,j in z])


plt.plot(x,y, 'o', label = "raw")
plt.plot(x - (x.max() + x.min()) / 2., y - (y.max() + y.min()) / 2., 'o', label = "corrected")
plt.xlim(-500, 500)
plt.ylim(-500, 500)
plt.title("calibration")
plt.xlabel("xMag")
plt.ylabel("yMag")
plt.show()
print (x.max() + x.min()) / 2., (y.max() + y.min()) / 2.

    
