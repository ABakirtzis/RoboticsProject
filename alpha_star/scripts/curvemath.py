from __future__ import division
import numpy as np
import time

#=======
#general
#=======

def which_cell(x, resolution, size): # cell of real coordinates x
    return (int((0.75 - x[1]) * resolution / size), int((x[0] + 0.75) * resolution / size))

def coords(x, resolution, size): # coordinates of cell x
    return (x[1] * size / resolution - 0.75, 0.75 - x[0] * size / resolution)

def neighbors2(a, resolution, board): # grid neighbors, for A* with diagonals
    ret = []
    for x,y in [(j,k) for j in range(-1, 2) for k in range(-1, 2) if j != 0 or k != 0]:
        if -1 < a[0] + x <= resolution and -1 < a[1] + y <= resolution and board[a[0]+x][a[1]+y] != 0:
            ret.append((x+a[0],y+a[1]))
    return ret

def neighbors(a, resolution, board): # grid neighbors, for A* without diagonals
    ret = []
    for x,y in [(j,k) for j in range(-1, 2) for k in range(-1, 2) if (j != 0 or k != 0) and j*k == 0]:
        if -1 < a[0] + x <= resolution and -1 < a[1] + y <= resolution and board[a[0]+x][a[1]+y] != 0:
            ret.append((x+a[0],y+a[1]))
    return ret

#======
#a_star
#======

def my_dist(a,b): # eucledian distance
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** (1/2.)


def reconstruct_path(cameFrom, current): # for A*
    total_path = [current]
    keyset = set(cameFrom.keys())
    while current in keyset:
        current = cameFrom[current]
        total_path.append(current)
    total_path.reverse()
    return total_path

def A_Star(board, start, goal, h, resolution): # the A* algorithm
    inf = resolution ** 3
    openSet = set([start])
    closedSet = set()
    cameFrom = {}
    gScore = {}
    gScore[start] = 0
    fScore = {}
    fScore[start] = h(start, goal)
    while len(openSet) > 0:
        current = min(openSet, key = lambda a: fScore.get(a, resolution ** 3))
        if current == goal:
            return reconstruct_path(cameFrom, current)
        openSet.remove(current)
        closedSet.add(current)
        for neighbor in neighbors(current, resolution, board):
            if neighbor in closedSet:
                continue
            tentative_gScore = gScore.get(current, inf) + h(current, neighbor)
            if neighbor not in openSet:
                openSet.add(neighbor)
            if tentative_gScore < gScore.get(neighbor, inf):
                cameFrom[neighbor] = current
                gScore[neighbor] = tentative_gScore
                fScore[neighbor] = gScore.get(neighbor, inf) + h(neighbor, goal)
    return None

#=========
#smoothing
#=========

def interpolate(x, y, t): # for a bezier curve
    return ((float(y[0]) - x[0]) * t + x[0], (float(y[1]) - x[1]) * t + x[1])


def bezier(p, t): # returns a point on a bezier curve given by p points with interpolation coefficient t
    while True:
        if len(p) == 2:
            return interpolate(p[0], p[1], t)
        p1 = []
        for i,j in zip(p[:-1], p[1:]):
            p1.append(interpolate(i, j, t))
        p = p1


def bezier_curve(p, n): # return n points of a bezier curve using a list of points p
    t = 1./n
    d = 0
    while d <= 1.001:
        yield bezier(p, d)
        d+=t

def bezier_curve2(p, n): #midpoints trick for big bezier curves
    s = p.pop(0)
    e = p.pop()
    c = []
    for i in range(len(p) - 1):
        c.append(((p[i][0] + p[i+1][0]) / 2, (p[i][1] + p[i+1][1]) / 2))
    p2 = []
    c1 = 0
    c2 = 0
    for i in range(len(p) + len(c)):
        if i % 2 == 0:
            p2.append(p[c1])
            c1 += 1
        else:
            p2.append(c[c2])
            c2 += 1
    p2 = [s] + p2 + [e]
    curve = []
    dall = 0
    for i in range(len(p2)-1):
        dall += ((p2[i][0] - p2[i+1][0]) ** 2 + (p2[i][1] - p2[i+1][1]) ** 2) ** (1/2.)
    for i in range(1, len(p2), 2):
        dc = ((p2[i][0] - p2[i+1][0]) ** 2 + (p2[i][1] - p2[i+1][1]) ** 2) ** (1/2.) + ((p2[i][0] - p2[i-1][0]) ** 2 + (p2[i][1] - p2[i-1][1]) ** 2) ** (1/2.)
        n2 = max(3, int(dc/dall*n))
        for j in bezier_curve(p2[i-1:i+2], n2):
            curve.append(j)
    return curve

def smoothen_curve(cur, smoothing_range, resolution, size): #curve smoothing
    p = []
    pi = []
    curves = []
    curvesi = []
    num = int(smoothing_range * resolution / size)
    if num < 3:
        return cur
    for i in range(1, len(cur) - 1):
        p1 = cur[i-1]
        p2 = cur[i]
        p3 = cur[i+1]
        if abs(np.arctan2(p2[1] - p1[1], p2[0] - p1[0]) - np.arctan2(p3[1] - p2[1], p3[0] - p2[0])) > np.pi/8:
            p.append(p2)
            pi.append(i)
    escflag = True
    for i in range(len(p)):
        if escflag:
            curves.append([])
            curvesi.append([])
            if pi[i] - num < 0:
                curves[-1].append(cur[0])
                curvesi[-1].append(0)
            elif len(curves) > 1 and pi[i] - num <= curvesi[-2][-1]:
                curves[-1].append(cur[curvesi[-2][-1] + 1])
                curvesi[-1].append(curvesi[-2][-1] + 1)
            else:
                curves[-1].append(cur[pi[i] - num])
                curvesi[-1].append(pi[i] - num)
        else:
            escflag = True
        curves[-1].append(p[i])
        curvesi[-1].append(pi[i])
        if pi[i] + num > len(cur) and i == len(p) - 1:
            curves[-1].append(cur[-1])
            curvesi[-1].append(len(cur)-1)
        elif i == len(p) - 1:
            curves[-1].append(cur[pi[i] + num])
            curvesi[-1].append(pi[i] + num)
        elif pi[i] + num >= pi[i+1]:
            escflag = False
        elif pi[i] + num < len(cur):
            curves[-1].append(cur[pi[i] + num])
            curvesi[-1].append(pi[i] + num)
        else:
            escflag = False
    rest = [cur[:curvesi[0][0]]]
    resti = [(0, curvesi[0][0])]
    for i in range(len(curvesi)-1):
        rest.append(cur[curvesi[i][-1]+1:curvesi[i+1][0]])
        resti.append((curvesi[i][-1]+1,curvesi[i+1][0]))
    final_curve = []
    if curvesi[0][0] > 0:
        final_curve = rest.pop(0)
    for i in range(len(curves)):
        final_curve += bezier_curve2(curves[i], curvesi[i][-1] - curvesi[i][0] + 1)
        if curvesi[i][-1] < len(cur)-1:
            final_curve += rest.pop(0)
    if len(rest) > 0:
        final_curve += rest.pop(0)
    return final_curve
        

def find_curve(start, end, obstacles, resolution = 80, size = 1.5, safety_net = 0.15, smoothing = True, smoothing_range = 0.08, debugging = False, plot_ret = False): # does A* and smooths curve
    if debugging or plot_ret:
        if debugging:
            from matplotlib import pyplot as plt
        plotobstacles = []
        plotobstacles_safety = []
        
    board = [[1 for i in range(resolution+1)] for j in range(resolution+1)]
    obstacles = [((min(i,j) - safety_net, max(i,j) + safety_net), (max(k,l) + safety_net, min(k,l) - safety_net)) for (i,j), (k,l) in obstacles]

    for i in obstacles:
        for k in range(int((i[0][0] + 0.75) * resolution / size), int((i[0][1] + 0.75) * resolution / size) + 1):
            for j in range(int((0.75 - i[1][0]) * resolution / size), int((0.75 - i[1][1]) * resolution / size) + 1):
                board[j][k] = 0
                if debugging or plot_ret:
                    if int((i[0][0] + 0.75 + safety_net) * resolution / size) < k < int((i[0][1] + 0.75 - safety_net) * resolution / size) + 1 and int((0.75 - i[1][0] + safety_net) * resolution / size) < j < int((0.75 - i[1][1] - safety_net) * resolution / size) + 1:
                        plotobstacles.append((j,k))
                    else:
                        plotobstacles_safety.append((j,k))
            
    curve = A_Star(board, which_cell(start, resolution, size), which_cell(end, resolution, size), my_dist, resolution)
    cur = [coords(i, resolution, size) for i in curve]
    p = smoothen_curve(cur, smoothing_range, resolution, size)
    if debugging or plot_ret:
        obs = [coords(i, resolution, size) for i in plotobstacles]
        obs_safety = [coords(i, resolution, size) for i in plotobstacles_safety]
        if debugging:
            plt.plot([i[0] for i in obs], [i[1] for i in obs], 'o', label = 'obstacle')
            plt.plot([i[0] for i in obs_safety], [i[1] for i in obs_safety], 'o', label = 'safety net')
            plt.plot([i[0] for i in cur], [i[1] for i in cur], 'o', label = 'A*')
            plt.xlim(-size/2, size/2)
            plt.ylim(-size/2, size/2)
            plt.xlabel('x(m)')
            plt.ylabel('y(m)')
            plt.title('Curve Smoothing')
        
            plt.plot([i[0] for i in p], [i[1] for i in p], markersize = 3, linewidth = 2, label = 'final curve')
            plt.legend()
            plt.show()

    if plot_ret:
        return p, obs, obs_safety
    return p


if __name__ == "__main__":
    find_curve(start = (0, 0.45), end = (0.30, -0.55), obstacles = [((0.30, 0.15), (-0.30, -0.15))], smoothing_range = 0.8, debugging = True )
