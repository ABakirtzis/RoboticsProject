from __future__ import division
import re
import sympy
from sympy.matrices import *
import numpy as np
import linemath
import time
import pickle
from sympy import *
from math import *

sampleaa = 0

#Mean value for 0.2 velocity
mean2 = 0.148

#Mean value for 0 velocity
mean0 = 0

#Standard deviation for 0.2 velocity
std2 = 0.00474

#Standard deviation for 0 velocity
std0 = 0.001


prevest = Matrix([0.45,0.45,-np.pi / 2])
angle_threshold = 25 * np.pi / 180
sensorApoklisi = 0.5
sonar_deviation_threshold = 0.3


[x,y,v,theta,w,t, dv] = sympy.symbol.symbols('x y v theta w t dv', real=True)

#Phi matrix of Kalman
Phi = Matrix([x + v * t * sympy.cos(theta), y + v * t * sympy.sin(theta), theta + w * t])
Phi_lambd = sympy.lambdify([x,y,theta, v, t, w], Phi)

#A matrix of Kalman
A = Phi.jacobian([x, y, theta])
A_lambd = sympy.lambdify([x,y,v,theta,t,w], A)

#Cw matrix of Kalman
Cw = Matrix([[dv**2*t**2*sympy.cos(theta)**2, dv**2*t**2*sympy.sin(theta)*sympy.cos(theta), 0], [dv**2*t**2*sympy.sin(theta)*sympy.cos(theta), dv**2*t**2*sympy.sin(theta)**2, 0], [0, 0, 0.0015**2*t**2]])
Cw_lambd = sympy.lambdify([theta, t, dv], Cw)

#Angles of sonars in relation to the robot's axis
sonarangles = [-np.pi/2, -np.pi/3, 0, np.pi/3, np.pi/2]

#Positions of sonars in relation to the robot's center (in our case the imu) in cm
sonarpoints = [(1.2, -3.5), (11.85, -2.25), (12.8, 0), (11.85, 2.25), (1.2, 3.5)] # in cm

sonarpoints = [(i * 0.01, j * 0.01) for i,j in sonarpoints] #in m

#Equations ax + by + c = 0 of lines of walls and 
#obstacles in [a,b,c] format (with and without obstacles)
walls = [[1, 0, -0.75], [0, 1, -0.75], [1, 0, 0.75], [0, 1, 0.75], linemath.line_from_points((0.15, -0.15), (0.30, -0.15)), linemath.line_from_points((0.15, -0.15), (0.15, -0.30)), linemath.line_from_points((0.15, -0.30), (0.30, -0.30)), linemath.line_from_points((0.30, -0.30), (0.30, -0.15))]
walls1 = [[1, 0, -0.75], [0, 1, -0.75], [1, 0, 0.75], [0, 1, 0.75]]

#Start and end points of walls and obstacles in (x,y) format
wallpoints = [[(0.75, -0.75), (0.75, 0.75)], [(0.75, 0.75), (-0.75, 0.75)], [(-0.75, 0.75), (-0.75, -0.75)], [(-0.75, -0.75), (0.75, -0.75)], [(0.15, -0.15), (0.30, -0.15)], [(0.15, -0.15), (0.15, -0.30)], [(0.15, -0.30), (0.30, -0.30)], [(0.30, -0.30), (0.30, -0.15)]]
wallpoints1 = [[(0.75, -0.75), (0.75, 0.75)], [(0.75, 0.75), (-0.75, 0.75)], [(-0.75, 0.75), (-0.75, -0.75)], [(-0.75, -0.75), (0.75, -0.75)]]

#Loading lambdified equations that get pos and angle
#of the robot and return sonar value (to accelerate evalf)
with open('/home/ubuntu/catkin_ws/src/state_estimation/scripts/equations', 'r') as fhandle:
    equations = pickle.load(fhandle)
with open('/home/ubuntu/catkin_ws/src/state_estimation/scripts/Hequations', 'r') as fhandle:
    Hequations = pickle.load(fhandle)

Hequations_lambd = []
equations_lambd = []

#Make H matrix of Kalman
for sonar_ind in range(0, 5):
    Hequations_lambd_temp = []
    equations_lambd_temp = []
    for wall_ind in range(0, len(walls)):
        equations_lambd_temp.append(sympy.lambdify([x,y,theta], equations[sonar_ind][wall_ind]))
        Hequations_lambd_temp.append(sympy.lambdify([x,y,theta], Hequations[sonar_ind][wall_ind]))
    Hequations_lambd.append(Hequations_lambd_temp)
    equations_lambd.append(equations_lambd_temp)

#Function to find [a,b,c] of equation ax + by + c = 0
#crossing (x0,y0) with angle theta0
def line(x0, y0, theta0):
    return [-np.sin(theta0), np.cos(theta0), -np.cos(theta0) * y0 + np.sin(theta0) * x0]

#Function to set angle in (-pi, pi)
def norm(b):
    if b > np.pi:
        return b - 2*np.pi
    if b <= -np.pi:
        return b + 2*np.pi
    return b

#Main Kalman function
def update(X, P, dt, sonars, vx, wz): #X = [x, y, theta]
    global prevest, sampleaa
    timestarted = time.time()
    
    #Set mean and standard deviation
    if vx > 0.1:
        velocity = mean2
        dvelocity = std2
    else:
        velocity = mean0
        dvelocity = std0
    
    #Evaluate Kalman matrices (A was evaluated faster with evalf)
    #others were evaluated faster with lambdify
    Ak = A.evalf(subs = {x:X[0], y:X[1], v:velocity, theta:X[2], t:dt, w:wz})

    Cwk = Cw_lambd(float(X[1]), float(dt), float(dvelocity))

    newPtemp = Ak * P * Ak.transpose() + Cwk

    est = Phi_lambd(float(X[0]), float(X[1]), float(X[2]), float(velocity), float(dt), float(wz))
    
    est[2] = norm(est[2])
    
    #If estimation is out of map, get previous estimation
    if abs(est[0]) > 0.7 or abs(est[1]) > 0.7:
        est[0] = prevest[0]
        est[1] = prevest[1]
    prevest = est

    heval, H, Cv, z = linemath.makeh_H_Cv_z(est[0], est[1], est[2], sonars, sonarangles, sonarpoints, walls, wallpoints, equations, Hequations, equations_lambd, Hequations_lambd)

    #Normalize difference of estimated angle
    if abs(z[-1] - est[2]) > np.pi:
        if z[-1] > est[2]:
            z[-1] -= 2 * np.pi
        else:
            est[2] -= 2 * np.pi

    newH = []
    for sonars_indx in range(len(H)-1):
        newH.append(H[sonars_indx](float(est[0]), float(est[1]), float(est[2])))

    newH.append(H[-1])
    newH = Matrix(newH)
    newK = (newPtemp * newH.transpose()) *  (newH * newPtemp * newH.transpose() + Cv).inv()
    
    zheval = z - heval
    zheval[-1] = norm(zheval[-1])
    newX = est + newK * (zheval)

    newX = np.array(newX).astype(np.float64)
    newX[2] = norm(newX[2])
    newX = sympy.Matrix(newX)
  
    sampleaa += 1

    newP = (eye(3) - newK * newH) * newPtemp

    newdt = time.time() - timestarted
    retX = Phi_lambd(float(newX[0]), float(newX[1]), float(newX[2]), float(velocity), float(newdt), float(wz))
    
    retX[2] = norm(est[2])

    return newX, newP, retX



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
