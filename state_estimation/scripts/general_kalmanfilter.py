from __future__ import division
import re
import sympy
from sympy.matrices import *
import numpy as np
import linemath

sampleaa = 0

mean2 = 0.148
mean0 = 0
std2 = 0.00474
std0 = 0.001
prevest = Matrix([0,0,0])
angle_threshold = 25 * np.pi / 180
sensorApoklisi = 0.5
sonar_deviation_threshold = 0.3


[x,y,v,theta,w,t, dv] = sympy.symbol.symbols('x y v theta w t dv')

Phi = Matrix([x + v * t * sympy.cos(theta), y + v * t * sympy.sin(theta), theta + w * t])

A = Phi.jacobian([x, y, theta])

#gw = Matrix([t * dv * sympy.cos(theta), t * sympy.sin(theta) * dv, t * dw])

Cw = Matrix([[dv**2*t**2*sympy.cos(theta)**2, dv**2*t**2*sympy.sin(theta)*sympy.cos(theta), 0], [dv**2*t**2*sympy.sin(theta)*sympy.cos(theta), dv**2*t**2*sympy.sin(theta)**2, 0], [0, 0, 0.0015**2*t**2]])


walls = [[1, 0, -0.75], [0, 1, -0.75], [1, 0, 0.75], [0, 1, 0.75]]
wallpoints = [[(0.75, -0.75), (0.75, 0.75)], [(0.75, 0.75), (-0.75, 0.75)], [(-0.75, 0.75), (-0.75, -0.75)], [(-0.75, -0.75), (0.75, -0.75)]]
sonarangles = [-np.pi/2, -np.pi/3, 0, np.pi/3, np.pi/2]
sonarpoints = [(1.2, -3.5), (11.85, -2.25), (12.8, 0), (11.85, 2.25), (1.2, 3.5)] # in cm
sonarpoints = [(i * 0.01, j * 0.01) for i,j in sonarpoints] #in m

#loading equations
with open('/home/ubuntu/catkin_ws/src/state_estimation/scripts/equations', 'r') as fhandle:
    equations = pickle.load(fhandle)
with open('/home/ubuntu/catkin_ws/src/state_estimation/scripts/Hequations', 'r') as fhandle:
    Hequations = pickle.load(fhandle)
#loading equations

print "ekei"
def line(x0, y0, theta0):
    return [-np.sin(theta0), np.cos(theta0), -np.cos(theta0) * y0 + np.sin(theta0) * x0]


#def intersection(l1, l2):
#    denom = l1[1]*l2[0] - l1[0]*l2[1]
#    if denom == 0:
#        return None
#    return ((l1[2]*l2[1] - l1[1]*l2[2]) / denom, (l2[2]*l1[0] - l2[0]*l1[2]) / denom)




def norm(b):
    if b > np.pi:
        return b - 2*np.pi
    if b <= -np.pi:
        return b + 2*np.pi
    return b

def update(X, P, dt, sonars, vx, wz): #X = [x, y, theta]
    global prevest, sampleaa

    print "mphka"

    if vx > 0.1:
        velocity = mean2
        dvelocity = std2
    else:
        velocity = mean0
        dvelocity = std0

    #print "evals"

    #print "lock eval"
    Ak = A.evalf(subs = {x:X[0], y:X[1], v:velocity, theta:X[2], t:dt, w:wz})
    #print "unlock"
    Cwk = Cw.evalf(subs = {theta:X[2], t:dt, dv:dvelocity})
    newPtemp = Ak * P * Ak.transpose() + Cwk


    est = Phi.evalf(subs = {x:X[0], y:X[1], v:velocity, theta:X[2], t:dt, w:wz})
    est[2] = norm(est[2])

    if abs(est[0]) > 0.6 or abs(est[1]) > 0.6:
        est[0] = prevest[0]
        est[1] = prevest[1]
    prevest = est
    print "gamithike"
    heval, H, Cv, z = linemath.makeh_H_Cv_z(est[0], est[1], est[2], sonars, sonarangles, sonarpoints, walls, wallpoints, equations, Hequations)

    
    if abs(z[-1] - est[2]) > np.pi:
        if z[-1] > est[2]:
            z[-1] -= 2 * np.pi
        else:
            est[2] -= 2 * np.pi
    print "gamietai"
            
    newH = H.evalf(subs = {x:est[0], y:est[1], theta:est[2]})
    ##print "newH:\n{}".format(newH)
    print "gamietai pio poli"
    sympy.pprint(H)
    newK = newPtemp * newH.transpose() * (newH * newPtemp * newH.transpose() + Cv).inv()
    print "giati einai toso argo"
    ##print "newK:\n{}\nz:\n{}\nh:\n{}\n, newK*:\n{}".format(newK, z, heval, newK*(z-heval))

    #print "z: {}\n heval: {}".format(z[:-1], heval[:-1]) 

    zheval = z - heval
    zheval[-1] = norm(zheval[-1])
    newX = est + newK * (zheval)

    print "heval: {}, z: {}".format(heval, z)
    
    newX[2] = norm(newX[2])
    print "sample: {}, x: {}, y: {}, a: {}, w: {}".format(sampleaa, newX[0], newX[1], newX[2] * 180 / np.pi, wz)
    sampleaa += 1

    newP = (eye(3) - newK * newH) * newPtemp

    return newX, newP



if __name__ == "__main__":
    """for i in range(5):
        for j in range(4):
            #print "def wall{}{}(x, y, theta):".format(i,j)
            #print '\treturn ({}, ['.format(str(equations[i][j]).replace("sin", "np.sin").replace("cos", "np.cos")),
            for k in [x,y,v,theta]:
                #print str(sympy.simplify(sympy.diff(equations[i][j], k))).replace("sin", "np.sin").replace("cos", "np.cos") + [",",""][k==theta],
            #print '])\n\n'"""
    #sympy.p#print(gw * gw.transpose())
    ##print (gw*gw.transpose())
    #print sympy.latex(Cw)
