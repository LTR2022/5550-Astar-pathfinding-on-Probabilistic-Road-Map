# Load the PIL, numpy, and heapq libraries
from PIL import Image, ImageDraw
import numpy as np
import heapq as q
from matplotlib.pyplot import imshow
import random

# Read image from disk using PIL
occupancy_map_img = Image.open('occupancy_map.png')

# Interpret this image as a numpy array, and threshold its values to {0,1}
occupancy_grid = (np.asarray(occupancy_map_img) > 0).astype(int)
occupancy_grid_probabilistic =occupancy_grid

# Assing the starting and goal
# g = (625, 150)
# g = (570, 150)
s = (635, 140)
g = (350, 400)

# set density of the sampling function
density = 0.08
# set size of local planer (pixel unit)
sizeRadius = 8

# Create a Vertex list
V = []
for i in range(occupancy_grid.shape[0]):
    for j in range(occupancy_grid.shape[1]):
        if occupancy_grid[i][j] == 1:
            V.append((i, j))
print("V[] is ready")

num_points = int(len(V) * density)
Vs = random.sample(V, num_points)
Vs.append((s[0],s[1]))
Vs.append((g[0],g[1]))

print("Vs[] is ready")

for i in range(len(Vs)):
    occupancy_grid_probabilistic[Vs[i][0]][Vs[i][1]]=2
print("occupancy_grid_probabilistic is ready")


def RecoverPath(s, g, pred):
    path = []
    path.insert(0, g)
    v = g
    while s not in path:
        path.insert(0, pred[v])
        v = pred[v]

    return path


flag = 1
def checkNeighbors(a,b):
    global flag
    flag = 1
    if b[0] >= a[0] and b[1] >= a[1]:
        recursionCheckNeighbors1(a, b)
    elif b[0] >= a[0] and b[1] <= a[1]:
        recursionCheckNeighbors2(a, b)
    elif b[0] <= a[0] and b[1] <= a[1]:
        recursionCheckNeighbors3(a, b)
    elif b[0] <= a[0] and b[1] >= a[1]:
        recursionCheckNeighbors4(a, b)

    return flag


# check if this neighbor can be reached in straight line
# recursion partition check all middle points
def recursionCheckNeighbors1(a, b):
    # check if this neighbor can be reached in straight line
    # recursion partition check all middle points
    check=[a[0]+int((b[0]-a[0]+1)/2), a[1]+int((b[1]-a[1]+1)/2)]
    # draw = ImageDraw.Draw(occupancy_map_img)
    # # draw.ellipse((check[0] - 2, check[1] - 2, check[0] + 2, check[1] + 2), outline=1)
    # draw.point([check[0], check[1]], fill=1)
    # occupancy_map_img.show()
    if occupancy_grid[check[1]][check[0]]==0:
         global flag
         flag = 0
    if check[0]>a[0] or check[1]>a[1]:
         recursionCheckNeighbors1(a,[check[0]-1,check[1]-1])
    if check[0]<b[0] or check[1]<b[1]:
         recursionCheckNeighbors1([check[0]+1,check[1]+1],b)

def recursionCheckNeighbors2(a, b):
    check=[a[0]+int((b[0]-a[0]+1)/2), b[1]+int((a[1]-b[1]+1)/2)]
    if occupancy_grid[check[1]][check[0]]==0:
         global flag
         flag = 0
    if check[0]>a[0] or check[1]<a[1]:
         recursionCheckNeighbors2(a,[check[0]-1,check[1]+1])
    if check[0]<b[0] or check[1]>b[1]:
         recursionCheckNeighbors2([check[0]+1,check[1]-1],b)

def recursionCheckNeighbors3(a, b):
    check=[b[0]+int((a[0]-b[0]+1)/2), b[1]+int((a[1]-b[1]+1)/2)]
    if occupancy_grid[check[1]][check[0]]==0:
         global flag
         flag = 0
    if check[0]<a[0] or check[1]<a[1]:
         recursionCheckNeighbors3(a,[check[0]+1,check[1]+1])
    if check[0]>b[0] or check[1]>b[1]:
         recursionCheckNeighbors3([check[0]-1,check[1]-1],b)

def recursionCheckNeighbors4(a, b):
    check=[b[0]+int((a[0]-b[0]+1)/2), a[1]+int((b[1]-a[1]+1)/2)]
    if occupancy_grid[check[1]][check[0]]==0:
         global flag
         flag = 0
    if check[0]<a[0] or check[1]>a[1]:
         recursionCheckNeighbors4(a,[check[0]+1,check[1]-1])
    if check[0]>b[0] or check[1]<b[1]:
         recursionCheckNeighbors4([check[0]-1,check[1]+1],b)



def N(v):
    neighbors = set()  # emprty neighbors set
    # check if +-sizeRadius * +-sizeRadius square neighbors vertices around the vertex v are in the occupancy grid
    for i in range(0-sizeRadius, sizeRadius+1):
        for j in range(0-sizeRadius, sizeRadius+1):
            a=int(v[0] + i)
            b=int(v[1] + j)
            if (occupancy_grid_probabilistic[v[0] + i][v[1] + j] == 2) and (i != 0 or j != 0) and (checkNeighbors([v[1], v[0]], [b, a]) == 1):
                neighbors.add((v[0] + i, v[1] + j))
                draw = ImageDraw.Draw(occupancy_map_img)
                draw.point([b, a], fill=1)
    return neighbors


def d(v1, v2):
    # return the euclidian distance between the two vertices
    point1 = np.array(v1)
    point2 = np.array(v2)
    return np.linalg.norm(point1 - point2)



def search(V, s, g, N, d):  # w() and h() are replaced with d()
    # Initialization
    CostTo = {}
    EstTotalCost = {}
    pred = {}
    for v in V:
        CostTo[v] = np.inf
        EstTotalCost[v] = np.inf

    CostTo[s] = 0
    EstTotalCost[s] = d(s, g)

    Q = []
    q.heapify(Q)
    q.heappush(Q, (d(s, g), s))

    # Main loop
    while len(Q) != 0:
        Qv = q.heappop(Q)
        vertex = Qv[1]
        if vertex == g:
            # Print the total length
            print(EstTotalCost[pred[g]])  # The last emstimation is the same as the total length

            path = RecoverPath(s, g, pred)

            # Plot the path on the image
            draw = ImageDraw.Draw(occupancy_map_img)
            for draw_idx in range(len(path) - 1):
                # flip the tuple element of the path list since draw.line recognizes the coordinates as (x,y)
                draw.line((path[draw_idx][1], path[draw_idx][0]) + (path[draw_idx + 1][1], path[draw_idx + 1][0]), fill="red")
            imshow(occupancy_map_img)

            return path

        for k in N(vertex):
            pvi = CostTo[vertex] + d(vertex, k)
            if pvi < CostTo[k]:
                # The path to i through v is better than the previously known best path to i,
                # so record it as the new best path to i
                pred[k] = vertex
                CostTo[k] = pvi
                EstTotalCost[k] = pvi + d(k, g)

                idx = 0
                m = []
                for l in range(len(Q)):
                    m.insert(0, Q[l][1])
                for l in m:
                    # Update i's priority
                    if k == l:
                        Q[idx] = (EstTotalCost[k], k)
                        break
                    idx = idx + 1
                # if i is not in Q
                if idx == len(Q):
                    q.heappush(Q, (EstTotalCost[k], k))

    # Print the total length
    print(CostTo[g])

    return {}  # Return empty set



if __name__ == '__main__':
    print("A* pathfinding start")
    search(Vs, s, g, N, d)
    occupancy_map_img.save('output.png', 'PNG')
    occupancy_map_img.show()
    print("finish")