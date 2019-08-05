from __future__ import division
from matplotlib import pyplot as plt
import numpy as np
import pdcon
import time
import linemath

resolution = 50
size = 1.5
safetynet = 0.1
curve_lookahead = 0.07
board = [[1 for i in range(resolution+1)] for j in range(resolution+1)]
start = (-0.15, -0.6)
end = (-0.30, 0.6)
obstacles = [((-0.30, -0.15), (0.30, 0.15))]

obstacles = [((min(i,j) - safetynet, max(i,j) + safetynet), (max(k,l) + safetynet, min(k,l) - safetynet)) for (i,j), (k,l) in obstacles]

plotobstacles = []

class EscapeLoop(Exception):
    pass

def which_cell(x):
    return (int((0.75 - x[1]) * resolution / size), int((x[0] + 0.75) * resolution / size))

def coords(x):
    return (x[1] * size / resolution - 0.75, 0.75 - x[0] * size / resolution)

for i in obstacles:
    for k in range(int((i[0][0] + 0.75) * resolution / size), int((i[0][1] + 0.75) * resolution / size) + 1):
        for j in range(int((0.75 - i[1][0]) * resolution / size), int((0.75 - i[1][1]) * resolution / size) + 1):
            board[j][k] = 0
            plotobstacles.append((j,k))


def neighbors(a):
    ret = []
    for x,y in [(j,k) for j in range(-1, 2) for k in range(-1, 2) if j != 0 or k != 0]:
        if -1 < a[0] + x <= resolution and -1 < a[1] + y <= resolution and board[a[0]+x][a[1]+y] != 0:
            ret.append((x+a[0],y+a[1]))
    return ret

def my_dist(a,b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** (1/2.)


def reconstruct_path(cameFrom, current):
    total_path = [current]
    keyset = set(cameFrom.keys())
    while current in keyset:
        current = cameFrom[current]
        total_path.append(current)
    total_path.reverse()
    return total_path

def A_Star(start, goal, h):
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
        for neighbor in neighbors(current):
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



curve = A_Star(which_cell(start), which_cell(end), my_dist)
#1, 30
#0.5, 40
pda = pdcon.pd_controller(setpoint = 0, kp = 2, kd = 0.1)
pd = pdcon.pd_controller(setpoint = 0, kp = 0.5, kd = 40)
obs = [coords(i) for i in plotobstacles]
cur = [coords(i) for i in curve]
x,y,a,v,w = start[0], start[1], np.arctan2(cur[1][1] - cur[0][1], cur[1][0] - cur[0][0]), 0.2, 0
positions = []
prevtime = time.time()
plt.plot([i[0] for i in obs], [i[1] for i in obs], 'o')
plt.plot([i[0] for i in cur], [i[1] for i in cur], 'o-')
plt.xlim(-0.75, 0.75)
plt.ylim(-0.75, 0.75)
while ((x - end[0]) ** 2 + (y - end[1]) ** 2) ** (1/2.) > 0.05:
    dt = (time.time() - prevtime)
    prevtime = time.time()
    m = 1000
    ind = 0
    for i in range(len(curve)):
        if my_dist(cur[i], (x,y)) < m:
            m = my_dist(cur[i], (x,y))
            ind = i
    if ind == 0:
        p1 = cur[ind]
        p2 = cur[ind+1]
    else:
        p2 = cur[ind]
        p1 = cur[ind-1]
    a += w * dt
    x += v * np.cos(a) * dt
    y += v * np.sin(a) * dt
    w = min(abs(pd.pd_out(m)), 6.5)
    if linemath.is_it_left(p1, p2, (x,y)):
        w = -w
    points_ahead = int(curve_lookahead / size * resolution)
    if points_ahead + ind + 1 < len(cur):
        p3 = cur[ind + points_ahead]
        p4 = cur[ind + points_ahead + 1]
        a1 = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
        a2 = np.arctan2(p4[1] - p3[1], p4[0] - p3[0])
        diffa = linemath.norm(a2 - a1)
        angout = pda.pd_out(diffa)
        print angout
        w += angout
    plt.plot(x,y, 'bo', linewidth = 0.1)
    plt.pause(0.1)

plt.show()
