from __future__ import division
import re
import sympy
from sympy.matrices import *
import numpy as np

mean2 = 0.0988927000344
mean0 = 0.00243190710785
std2 = 0.0675333972387
std0 = 0.00466078563696
prevest = Matrix([0,0,0])

sensorApoklisi = 0.5

[x,y,v,theta,w,t, dv] = sympy.symbol.symbols('x y v theta w t dv')


Phi = Matrix([x + v * t * sympy.cos(theta), y + v * t * sympy.sin(theta), theta + w * t])

A = Phi.jacobian([x, y, theta])

#gw = Matrix([t * dv * sympy.cos(theta), t * sympy.sin(theta) * dv, t * dw])

Cw = Matrix([[dv**2*t**2*sympy.cos(theta)**2, dv**2*t**2*sympy.sin(theta)*sympy.cos(theta), 0], [dv**2*t**2*sympy.sin(theta)*sympy.cos(theta), dv**2*t**2*sympy.sin(theta)**2, 0], [0, 0, 0.002**2*t**2]])


walls = [[1, 0, -2], [0, 1, -2], [1, 0, 2], [0, 1, 2]]

equations = [[(2 - (x + 0.1 * sympy.cos(theta))) / sympy.cos(theta - np.pi/2) - 0.05, (2 - (y + 0.1 * sympy.sin(theta))) / sympy.sin(theta - np.pi/2) - 0.05, -(2 + (x + 0.1 * sympy.cos(theta))) / sympy.cos(theta - np.pi/2) - 0.05, -(2 + (y + 0.1 * sympy.sin(theta))) / sympy.sin(theta - np.pi/2) - 0.05],
             [(2 - (x + 0.1 * sympy.cos(theta))) / sympy.cos(theta - np.pi/4) - 0.05*np.sqrt(2), (2 - (y + 0.1 * sympy.sin(theta))) / sympy.sin(theta - np.pi/4) - 0.05*np.sqrt(2), -(2 + (x + 0.1 * sympy.cos(theta))) / sympy.cos(theta - np.pi/4) - 0.05*np.sqrt(2), -(2 + (y + 0.1 * sympy.sin(theta))) / sympy.sin(theta - np.pi/4) - 0.05*np.sqrt(2)],
             [(2 - (x + 0.1 * sympy.cos(theta))) / sympy.cos(theta) - 0.05, (2 - (y + 0.1 * sympy.sin(theta))) / sympy.sin(theta) - 0.05, -(2 + (x + 0.1 * sympy.cos(theta))) / sympy.cos(theta) - 0.05, -(2 + (y + 0.1 * sympy.sin(theta))) / sympy.sin(theta) - 0.05],
             [(2 - (x + 0.1 * sympy.cos(theta))) / sympy.cos(theta + np.pi/4) - 0.05*np.sqrt(2), (2 - (y + 0.1 * sympy.sin(theta))) / sympy.sin(theta + np.pi/4) - 0.05*np.sqrt(2), -(2 + (x + 0.1 * sympy.cos(theta))) / sympy.cos(theta + np.pi/4) - 0.05*np.sqrt(2), -(2 + (y + 0.1 * sympy.sin(theta))) / sympy.sin(theta + np.pi/4) - 0.05*np.sqrt(2)],
             [(2 - (x + 0.1 * sympy.cos(theta))) / sympy.cos(theta + np.pi/2) - 0.05, (2 - (y + 0.1 * sympy.sin(theta))) / sympy.sin(theta + np.pi/2) - 0.05, -(2 + (x + 0.1 * sympy.cos(theta))) / sympy.cos(theta + np.pi/2) - 0.05, -(2 + (y + 0.1 * sympy.sin(theta))) / sympy.sin(theta + np.pi/2) - 0.05]]

def line(x0, y0, theta0):
    return [-np.sin(theta0), np.cos(theta0), -np.cos(theta0) * y0 + np.sin(theta0) * x0]


def intersection(l1, l2):
    denom = l1[1]*l2[0] - l1[0]*l2[1]
    if denom == 0:
        return None
    return ((l1[2]*l2[1] - l1[1]*l2[2]) / denom, (l2[2]*l1[0] - l2[0]*l1[2]) / denom)


def valid_intersection(x0, y0, theta0, i):
    return -2 <= i[0] <= 2 and -2 <= i[1] <= 2 and (i[0] - x0) * np.cos(theta0) >= 0 and (i[1] - y0) * np.sin(theta0) >= 0


def which_wall(x0, y0, theta0):
    x0,y0,theta0 = float(x0),float(y0),float(theta0)
    myline = line(x0, y0, theta0)
    for i in range(4):
        p = intersection(myline, walls[i])
        if valid_intersection(x0, y0, theta0, p):
            return i
    return None


def makeh_H_Cv_z(x0, y0, theta0, sonars):
    sonarangles = [-np.pi/2, -np.pi/4, 0, np.pi/4, np.pi/2]
    h = []
    z = []
    for i in range(5):
    #    print "Checking sonar " + str(i)
        if sonars[i] > 1.99:
    #        print "Doesn't see"
            continue
        wall = which_wall(x0, y0, theta0 + sonarangles[i])
        print "sensor {} -> wall {}".format(i, wall)
        if wall == None:
            print i, x0, y0, theta0
            continue
        h.append(equations[i][wall])
        z.append(sonars[i])
    h.append(theta)
    h = Matrix(h)
    z.append(sonars[-1])
    z = Matrix(z)
    Cv = eye(len(h)) * 0.01 ** 2
    Cv[-1,-1] = 0.002 ** 2
    return (h, h.jacobian([x,y,theta]), Cv, z)


def norm(b):
    if b > np.pi:
        return b - 2*np.pi
    if b <= -np.pi:
        return b + 2*np.pi
    return b

def update(X, P, dt, sonars, vx, wz): #X = [x, y, theta]
    if vx > 0.1:
        velocity = mean2
        dvelocity = std2
    else:
        velocity = mean0
        dvelocity = std0
    Ak = A.evalf(subs = {x:X[0], y:X[1], v:velocity, theta:X[2], t:dt, w:wz})
    Cwk = Cw.evalf(subs = {theta:X[2], t:dt, dv:dvelocity})
    newPtemp = Ak * P * Ak.transpose() + Cwk

    est = Phi.evalf(subs = {x:X[0], y:X[1], v:velocity, theta:X[2], t:dt, w:wz})

    est[2] = norm(est[2])

    if abs(est[0]) > 1.89 or abs(est[1]) > 1.89:
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
    #print "newH:\n{}".format(newH)

    newK = newPtemp * newH.transpose() * (newH * newPtemp * newH.transpose() + Cv).inv()

    heval = h.evalf(subs = {x:est[0], y:est[1], theta:est[2]})
    #print "newK:\n{}\nz:\n{}\nh:\n{}\n, newK*:\n{}".format(newK, z, heval, newK*(z-heval))

    print "z: {}\n heval: {}".format(z[:-1], heval[:-1]) 

    newX = est + newK * (z - heval)
    
    newX[2] = norm(newX[2])

    newP = (eye(3) - newK * newH) * newPtemp

    return (newX, newP)
    


if __name__ == "__main__":
    """for i in range(5):
        for j in range(4):
            print "def wall{}{}(x, y, theta):".format(i,j)
            print '\treturn ({}, ['.format(str(equations[i][j]).replace("sin", "np.sin").replace("cos", "np.cos")),
            for k in [x,y,v,theta]:
                print str(sympy.simplify(sympy.diff(equations[i][j], k))).replace("sin", "np.sin").replace("cos", "np.cos") + [",",""][k==theta],
            print '])\n\n'"""
    #sympy.pprint(gw * gw.transpose())
    #print (gw*gw.transpose())
    print sympy.latex(Cw)
