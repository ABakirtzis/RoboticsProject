from __future__ import division
import sympy
from sympy.matrices import *
import numpy as np


x, y, theta = sympy.symbols('x y theta')
#needed variables:
#walls
#wallpoints
#sonarpoints
#sonarangles


#general line math

def line(x0, y0, theta0):
    return [-sympy.sin(theta0), sympy.cos(theta0), -sympy.cos(theta0) * y0 + sympy.sin(theta0) * x0]


def line_from_points(p1, p2):
    return (p2[1] - p1[1], p1[0] - p2[0], (p2[0] - p1[0]) * p1[1] + (p1[1] - p2[1]) * p1[0])


def line_point_dist(l, p):
    return abs(l[0] * p[0] + l[1] * p[1] + l[2]) / (l[0] ** 2 + l[1] ** 2) ** (1/2)


def point_point_dist(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** (1/2.)

def line_vec(l): # vector parallel to line with norm 1
    a,b,c = l
    if b != 0:
        p1 = np.array([0, -c])
        p2 = np.array([1, -a/b - c])
    else:
        p1 = np.array([-c, 0])
        p2 = np.array([-b/a - c, 1])
    r = p2 - p1
    return r / sympy.sqrt(r[0] ** 2 + r[1] ** 2)


def line_angle_sin(l1, l2):
    r1 = line_vec(l1)
    r2 = line_vec(l2)
    a = np.dot(r1,r2)
    return sympy.sqrt(1 - a ** 2)


def intersection(l11, l22):
    l1 = [float(i) for i in l11]
    l2 = [float(i) for i in l22]
    denom = l1[1]*l2[0] - l1[0]*l2[1]
    if denom == 0:
        return None
    return ((l1[2]*l2[1] - l1[1]*l2[2]) / denom, (l2[2]*l1[0] - l2[0]*l1[2]) / denom)


def valid_intersection(x0, y0, theta0, i, wall_limits):
    return min(wall_limits[0][0]) <= i[0] <= max(wall_limits[1][0]) and min(wall_limits[0][1]) <= i[1] <= max(wall_limits[1][1]) and (i[0] - x0) * np.cos(theta0) >= 0 and (i[1] - y0) * np.sin(theta0) >= 0


def perpendicular_line_from_point(l, p):
    return [-l[1], l[0], l[1] * p[0], l[0] * p[1]]


def is_it_left(p1, p2, p): # if p is left of the vector p1 --> p2
    rv = (p1[1] - p2[1], p2[0] - p1[0])
    r1 = (p[0] - p1[0], p[1] - p1[1])
    dot = rv[0] * r1[0] + rv[1] * r1[1]
    return dot > 0


def point_between_parallel_lines(l1, l2, p):
    a = l1[0] * p[0] + l1[1] * p[1] + l1[2]
    b = l2[0] * p[0] + l2[1] * p[1] + l2[2]
    return a * b < 0

#curve math

#curve = [p1, p2, ..., pn]

def valid_segment(p1, p2, p):
    l = line_from_points(p1,p2)
    e1 = perpendicular_line_from_point(l, p1)
    e2 = perpendicular_line_from_point(l, p2)
    return point_between_parallel_lines(e1, e2, p)


#kalman filter equation making


def sonar_pos(sonar_point):
    return x + sympy.cos(theta) * sonar_point[0], y + sympy.sin(theta) * sonar_point[1]


def sonar_pos_line(sonar_point, sonar_angle):
    p = sonar_pos(sonar_point)
    return p, line(p[0], p[1], theta + sonar_angle)


def wall_equation(sonar_point, sonar_angle, wall):
    p, l = sonar_pos_line(sonar_point, sonar_angle)
    c = line_angle_sin(l, wall)
    d = line_point_dist(wall, p)
    return sympy.simplify(d/c)


def make_equations(sonar_points, sonar_angles, walls):
    equations = [[wall_equation(sonar_points[j], sonar_angles[j], walls[i]) for i in walls] for j in range(len(sonar_points))]
    Hequations = [[[sympy.diff(i, k) for k in (x, y, theta)] for i in j] for j in equations]
    return equations, Hequations


def which_wall(x0, y0, theta0, walls):
    x0,y0,theta0 = float(x0),float(y0),float(theta0)
    myline = line(x0, y0, theta0)
    inter = None
    angle = None
    dist = None
    for i in range(len(walls)):
        p = intersection(myline, walls[i])
        if valid_intersection(x0, y0, theta0, p):
            tempdist = point_point_dist(p, (x0, y0))
            if inter == None:
                inter = i
                d = line_point_dist(walls[i], (x0, y0))
                angle = np.arccos(float(d)/float(tempdist))
                continue
            if tempdist < dist:
                dist = tempdist
                inter = i
                d = line_point_dist(walls[i], (x0, y0))
                d1 = ((p[0] - x0) ** 2 + (p[1] - y0) ** 2) ** (1/2)
                angle = np.arccos(float(d)/float(tempdist))
    return inter, angle

def makeh_H_Cv_z(x0, y0, theta0, sonars, sonarangles, sonarpoints, walls, equations, Hequations):
    theta0 = float(theta0)
    h = []
    z = []
    H = []
    heval = []
    for i in range(5):
        if sonars[i] > 2.3:
            continue
        wall, angle = which_wall(sonarpoints[i][0], sonarpoints[i][1], theta0 + sonarangles[i], walls)
        if angle > angle_threshold:
            continue
        if wall == None:
            continue
        eq = equations[i][wall].evalf(subs = {x: x0, y: y0, theta: theta0})
        if abs(sonars[i] - eq) < sonar_deviation_threshold:
            h.append(equations[i][wall])
            z.append(sonars[i])
            heval.append(eq)
            H.append(Hequations[i][wall])
    h.append(theta)
    heval.append(theta0)
    h = Matrix(h)
    z.append(sonars[-1])
    z = Matrix(z)
    heval = Matrix(heval)
    H.append([0, 0, 1])
    H = Matrix(H)
    Cv = eye(len(h)) * 0.03 ** 2
    Cv[-1,-1] = 0.0043** 2
    return (heval, H, Cv, z)


if __name__ == "__main__":
    walls = [[1, 0, -0.75], [0, 1, -0.75], [1, 0, 0.75], [0, 1, 0.75]]
    sonarangles = [-np.pi/2, -np.pi/3, 0, np.pi/3, np.pi/2]
    sonarpoints = [(1.2, -3.5), (11.85, -2.25), (12.8, 0), (11.85, 2.25), (1.2, 3.5)] # in cm
    sonarpoints = [(i * 0.01, j * 0.01) for i,j in sonarpoints] #in m
    eq = wall_equation(sonarpoints[0], sonarangles[0], walls[0])
    eq2 = (0.75 - x - sonarpoints[0][0] * sympy.cos(theta)) / sympy.cos(theta + sonarangles[0])
    for x1 in range(10):
        for y1 in range(10):
            for theta1 in range(30, 90, 5):
                print eq.evalf(subs = {x: x1 * 0.46 - 2.3, y: y1 * 0.46 - 2.3, theta: theta1 * sympy.pi / 180}) - eq2.evalf(subs = {x: x1 * 0.46 - 2.3, y: y1 * 0.46 - 2.3, theta: theta1 * sympy.pi / 180})
