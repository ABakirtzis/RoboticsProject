from __future__ import division
import re
import sympy
from sympy.matrices import *
import numpy as np

mean2 = 0.148
mean0 = 0
std2 = 0.00474
std0 = 0.001
prevest = Matrix([0,0,0])
angle_threshold = 25 * np.pi / 180
sensorApoklisi = 0.5

[x,y,v,theta,w,t, dv] = sympy.symbol.symbols('x y v theta w t dv')

Phi = Matrix([x + v * t * sympy.cos(theta), y + v * t * sympy.sin(theta), theta + w * t])

A = Phi.jacobian([x, y, theta])

#gw = Matrix([t * dv * sympy.cos(theta), t * sympy.sin(theta) * dv, t * dw])

Cw = Matrix([[dv**2*t**2*sympy.cos(theta)**2, dv**2*t**2*sympy.sin(theta)*sympy.cos(theta), 0], [dv**2*t**2*sympy.sin(theta)*sympy.cos(theta), dv**2*t**2*sympy.sin(theta)**2, 0], [0, 0, 0.0015**2*t**2]])


walls = [[1, 0, -0.75], [0, 1, -0.75], [1, 0, 0.75], [0, 1, 0.75]]
sonarangles = [-np.pi/2, -np.pi/3, 0, np.pi/3, np.pi/2]
sonarpoints = [(1.2, -3.5), (11.85, -2.25), (12.8, 0), (11.85, 2.25), (1.2, 3.5)] # in cm
sonarpoints = [(i * 0.01, j * 0.01) for i,j in sonarpoints] #in m

equations = [[(0.75 - x - sonarpoints[i][0] * sympy.cos(theta)) / sympy.cos(theta + sonarangles[i]),
              (0.75 - y - sonarpoints[i][1] * sympy.sin(theta)) / sympy.sin(theta + sonarangles[i]),
              (0.75 + x + sonarpoints[i][0] * sympy.cos(theta)) / sympy.cos(theta + sonarangles[i]),
              (0.75 + y + sonarpoints[i][1] * sympy.sin(theta)) / sympy.sin(theta + sonarangles[i])] for i in range(5)]


def line(x0, y0, theta0):
    return [-np.sin(theta0), np.cos(theta0), -np.cos(theta0) * y0 + np.sin(theta0) * x0]


def intersection(l1, l2):
    denom = l1[1]*l2[0] - l1[0]*l2[1]
    if denom == 0:
        return None
    return ((l1[2]*l2[1] - l1[1]*l2[2]) / denom, (l2[2]*l1[0] - l2[0]*l1[2]) / denom)


def valid_intersection(x0, y0, theta0, i):
    return -0.75 <= i[0] <= 0.75 and -0.75 <= i[1] <= 0.75 and (i[0] - x0) * np.cos(theta0) >= 0 and (i[1] - y0) * np.sin(theta0) >= 0


def line_point_dist(l, p):
    return abs(l[0] * p[0] + l[1] * p[1] + l[2]) / (l[0] ** 2 + l[1] ** 2) ** (1/2)


def which_wall(x0, y0, theta0):
    x0,y0,theta0 = float(x0),float(y0),float(theta0)
    myline = line(x0, y0, theta0)
    for i in range(4):
        p = intersection(myline, walls[i])
        if valid_intersection(x0, y0, theta0, p):
            d = line_point_dist(walls[i], (x0, y0))
            d1 = ((p[0] - x0) ** 2 + (p[1] - y0) ** 2) ** (1/2)
            angle = np.arccos(d/d1)
            return i, angle
    return None, None


def makeh_H_Cv_z(x0, y0, theta0, sonars):
    theta0 = float(theta0)
    sonarangles = [-np.pi/2, -np.pi/3, 0, np.pi/3, np.pi/2]
    sonarpoints1 = [(x0 + i * np.cos(theta0), y0 + j * np.sin(theta0)) for i,j in sonarpoints]
    h = []
    z = []
    for i in range(5):
    #    #print "Checking sonar " + str(i)
        if sonars[i] > 1.99:
    #        #print "Doesn't see"
            continue
        wall, angle = which_wall(sonarpoints1[i][0], sonarpoints1[i][1], theta0 + sonarangles[i])
        if angle > angle_threshold:
            continue
        #print "sensor {} -> wall {}".format(i, wall)
        if wall == None:
            #print i, sonarpoints1[i][0], sonarpoints1[i][1], theta0
            continue
        h.append(equations[i][wall])
        z.append(sonars[i])
    h.append(theta)
    h = Matrix(h)
    z.append(sonars[-1])
    z = Matrix(z)
    Cv = eye(len(h)) * 0.03 ** 2
    Cv[-1,-1] = 0.0043** 2
    return (h, h.jacobian([x,y,theta]), Cv, z)


def norm(b):
    if b > np.pi:
        return b - 2*np.pi
    if b <= -np.pi:
        return b + 2*np.pi
    return b

def update(X, P, dt, sonars, vx, wz): #X = [x, y, theta]

    #print "mphka"

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

    h, H, Cv, z = makeh_H_Cv_z(est[0], est[1], est[2], sonars)

    
    if abs(z[-1] - est[2]) > np.pi:
        if z[-1] > est[2]:
            z[-1] -= 2 * np.pi
        else:
            est[2] -= 2 * np.pi
            
    newH = H.evalf(subs = {x:est[0], y:est[1], theta:est[2]})
    ##print "newH:\n{}".format(newH)

    newK = newPtemp * newH.transpose() * (newH * newPtemp * newH.transpose() + Cv).inv()

    heval = h.evalf(subs = {x:est[0], y:est[1], theta:est[2]})
    ##print "newK:\n{}\nz:\n{}\nh:\n{}\n, newK*:\n{}".format(newK, z, heval, newK*(z-heval))

    #print "z: {}\n heval: {}".format(z[:-1], heval[:-1]) 

    newX = est + newK * (z - heval)
    
    newX[2] = norm(newX[2])

    newP = (eye(3) - newK * newH) * newPtemp

    return (newX, newP)
    


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
