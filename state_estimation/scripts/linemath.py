import sympy
from sympy.matrices import *
import numpy as np
import pickle


x, y, theta = sympy.symbols('x y theta', real=True) # used to make the equations
#needed variables:
#walls
#wallpoints
#sonarpoints
#sonarangles

angle_threshold = 25 * np.pi / 180 # after that, the sonars don't see
sonar_deviation_threshold = 0.3 # a difference greater than this between the measurement and the model estimation should be discarded
walls = [[1, 0, -0.75], [0, 1, -0.75], [1, 0, 0.75], [0, 1, 0.75]] # line equations of the walls
wallpoints = [[(0.75, -0.75), (0.75, 0.75)], [(0.75, 0.75), (-0.75, 0.75)], [(-0.75, 0.75), (-0.75, -0.75)], [(-0.75, -0.75), (0.75, -0.75)]] # end points of the walls
sonarangles = [-np.pi/2, -np.pi/3, 0, np.pi/3, np.pi/2] # see angles of sonars
sonarpoints = [(1.2, -3.5), (11.85, -2.25), (12.8, 0), (11.85, 2.25), (1.2, 3.5)] # in cm
sonarpoints = [(i * 0.01, j * 0.01) for i,j in sonarpoints] #in m


def line_from_points(p1, p2): # equation of line passing from two points
    return [p2[1] - p1[1], p1[0] - p2[0], (p2[0] - p1[0]) * p1[1] + (p1[1] - p2[1]) * p1[0]]


walls_obst = [[1, 0, -0.75], [0, 1, -0.75], [1, 0, 0.75], [0, 1, 0.75], line_from_points((0.15, -0.15), (0.30, -0.15)), line_from_points((0.15, -0.15), (0.15, -0.30)), line_from_points((0.15, -0.30), (0.30, -0.30)), line_from_points((0.30, -0.30), (0.30, -0.15))]


wall_points = [[(0.75, -0.75), (0.75, 0.75)], [(0.75, 0.75), (-0.75, 0.75)], [(-0.75, 0.75), (-0.75, -0.75)], [(-0.75, -0.75), (0.75, -0.75)], [(0.15, -0.15), (0.30, -0.15)], [(0.15, -0.15), (0.15, -0.30)], [(0.15, -0.30), (0.30, -0.30)], [(0.30, -0.30), (0.30, -0.15)]]

#general line math

def line(x0, y0, theta0): # equation of line passing from point x0, y0 having angle theta0
    return [-sympy.sin(theta0), sympy.cos(theta0), -sympy.cos(theta0) * y0 + sympy.sin(theta0) * x0]


def line_point_dist(l, p): # distance of a point p from a line l
    return abs(l[0] * p[0] + l[1] * p[1] + l[2]) / (l[0] ** 2 + l[1] ** 2) ** (1/2)


def point_point_dist(p1, p2): # distance between two points
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


def line_angle_sin(l1, l2): # convex angle sin between lines l1, l2
    r1 = line_vec(l1)
    r2 = line_vec(l2)
    a = np.dot(r1,r2)
    return sympy.sqrt(1 - a ** 2)


def intersection(l11, l22): # intersection of lines l11, l22
    l1 = [float(i) for i in l11]
    l2 = [float(i) for i in l22]
    denom = l1[1]*l2[0] - l1[0]*l2[1]
    if denom == 0:
        return None
    return ((l1[2]*l2[1] - l1[1]*l2[2]) / denom, (l2[2]*l1[0] - l2[0]*l1[2]) / denom)


def valid_intersection(x0, y0, theta0, i, wall_limits):
    # check if an intersection of a sonar line and a line is in the
    # line segment of the wall and at the right direction
    return min(wall_limits[0][0], wall_limits[1][0]) <= i[0] <= max(wall_limits[0][0], wall_limits[1][0]) and min(wall_limits[0][1], wall_limits[1][1]) <= i[1] <= max(wall_limits[0][1], wall_limits[1][1]) and (i[0] - x0) * np.cos(theta0) >= 0 and (i[1] - y0) * np.sin(theta0) >= 0


def perpendicular_line_from_point(l, p):
    # equation of line perpendicular to line l passing from point p
    return [-l[1], l[0], l[1] * p[0], l[0] * p[1]]


def is_it_left(p1, p2, p): # if p is left of the vector p1 --> p2
    rv = (p1[1] - p2[1], p2[0] - p1[0])
    r1 = (p[0] - p1[0], p[1] - p1[1])
    dot = rv[0] * r1[0] + rv[1] * r1[1]
    return dot > 0


def point_between_parallel_lines(l1, l2, p): # check if a point is between l1, l2
    a = l1[0] * p[0] + l1[1] * p[1] + l1[2]
    b = l2[0] * p[0] + l2[1] * p[1] + l2[2]
    return a * b < 0

#curve math

#curve = [p1, p2, ..., pn]

def valid_segment(p1, p2, p): # never used
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
    # compute the equation to add in the h table of the kalman filter
    # for a given sonar and the wall
    p, l = sonar_pos_line(sonar_point, sonar_angle)
    c = line_angle_sin(l, wall)
    d = line_point_dist(wall, p)
    return d/c

counter = 0


def mytrigsimp(a):
    global counter
    counter += 1
    return sympy.simplify(a)


def make_equations(sonar_points, sonar_angles, walls): # make all equations of h and all equations of H
    equations = [[sympy.simplify(wall_equation(sonar_points[j], sonar_angles[j], i)) for i in walls] for j in range(len(sonar_points))]
    print(equations)
    Hequations = [[[mytrigsimp(sympy.diff(i, k)) for k in (x, y, theta)] for i in j] for j in equations]
    print(Hequations)
    return equations, Hequations


def which_wall(x0, y0, theta0, walls, wall_limits, equations, Hequations):
    # find which wall is seen by the sonar assuming the robot is at x0, y0, theta0
    x0,y0,theta0 = float(x0),float(y0),float(theta0)
    myline = line(x0, y0, theta0)
    inter = None
    angle = None
    dist = None
    for i in range(len(walls)):
        p = intersection(myline, walls[i])
        if valid_intersection(x0, y0, theta0, p, wall_limits[i]):
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

def makeh_H_Cv_z(x0, y0, theta0, sonars, sonarangles, sonarpoints, walls, wall_limits, equations, Hequations, equations_lambd, Hequations_lambd):
    # if some sonars aren't in range or have great angles with a wall, don't put them in the H and h matrices
    theta0 = float(theta0)
    h = []
    z = []
    H = []
    heval = []
    for i in range(5):
      
        if sonars[i] > 2.3:
            continue
        wall, angle = which_wall(sonarpoints[i][0], sonarpoints[i][1], theta0 + sonarangles[i], walls, wall_limits, equations, Hequations)
        if angle > angle_threshold:
            continue
        if wall == None:
            continue
        eq = equations_lambd[i][wall](float(x0), float(y0), float(theta0))
        if abs(sonars[i] - eq) < sonar_deviation_threshold:
            h.append(equations[i][wall])
            z.append(sonars[i])
            heval.append(eq)
            H.append(Hequations_lambd[i][wall])
            
    h.append(theta)
    heval.append(theta0)
    h = Matrix(h)
    z.append(sonars[-1])
    z = Matrix(z)
    heval = Matrix(heval)
    H.append([0, 0, 1])
    Cv = eye(len(h)) * 0.03 ** 2
    Cv[-1,-1] = 0.0043** 2
    return (heval, H, Cv, z)


if __name__ == "__main__":
    equations, Hequations = make_equations(sonarpoints, sonarangles, walls_obst)
    with file("equations", 'w') as fhandle:
        pickle.dump(equations, fhandle)
    with file("Hequations", 'w') as fhandle:
        pickle.dump(Hequations, fhandle)
