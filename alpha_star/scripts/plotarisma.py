from __future__ import division
import numpy as np
from matplotlib import pyplot as plt


resolution = 50
size = 1.5
safetynet = 0.15
curve_lookahead = 0.07
board = [[1 for i in range(resolution+1)] for j in range(resolution+1)]
start = (0.45, 0.45)
end = (0.30, -0.55)
startang_robot = -np.pi/2
obstacles = [((0.30, 0.15), (-0.30, -0.15))]

obstacles = [((min(i,j) - safetynet, max(i,j) + safetynet), (max(k,l) + safetynet, min(k,l) - safetynet)) for (i,j), (k,l) in obstacles]

plotobstacles = []

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
obs = [coords(i) for i in plotobstacles]
cur = [coords(i) for i in curve]
with open("info", 'r') as fhandle:
    z = fhandle.read().split('\n')[:-1]
    x = [float(i.split(' ')[0].split(';')[0]) for i in z]
    y = [float(i.split(' ')[0].split(';')[1]) for i in z]

plt.plot([i[0] for i in obs], [i[1] for i in obs], 'o')
plt.plot([i[0] for i in cur], [i[1] for i in cur], 'o')
plt.plot(x, y, 'o')
plt.xlim(-0.75, 0.75)
plt.ylim(-0.75, 0.75)
plt.show()
