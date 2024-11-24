import pygame
import random
import heapq
import math
import time
import collections


def manhattanDistance(x1, y1, x2, y2):
    '''
    Sum of the X distance & Y distance
    '''
    return abs(y2-y1) + abs(x2-x1)


def diagonalDistance(x1, y1, x2, y2):
    '''
    Closest distance between two nodes(Pythagorean)
    '''
    return math.sqrt((y2-y1)**2+(x2-x1)**2)


def aStarHeuristic(x1, y1, x2, y2):
    '''
    Used to estimate the shortest path for the A* Algorithm
    Multiplies the manhattan distance with a weight
    '''
    return heuristicWeight*(manhattanDistance(x1, y1, x2, y2))


def solvePath(grid, draw=False):
    '''
    A* algorithm
    This algorithm visits every node in the graph similarly to Dijkstra’s algorithm, but visits them in an optimized order
    It is still able to find the shortest path, but the the time taken can be reduced by up to 50%
    '''
    directions = {(0, 1), (1, 0), (0, -1), (-1, 0)}
    #Stores distances and parent nodes
    distList = {(i, j): [float("inf"), -1] for i in range(gridSize[0]) for j in range(gridSize[1])}
    distList[startNode] = [0, startNode]
    visited = set()
    #Priority queue used for A*
    pQueue = [(0, startNode)]
    heapq.heapify(pQueue)

    counter = 0
    while len(pQueue) > 0:
        #Pops out the node with the smallest value (Distance from start + Distance to end * Heurstic weight)
        distance, node = heapq.heappop(pQueue)
        if node == endNode:
            return True, distList
        x, y = node
        if node not in visited:
            for direction in directions:
                #Visits each direction, and adds the visited node to the queue
                newX, newY = x + direction[0], y + direction[1]
                if 0 <= newX < gridSize[0] and 0 <= newY < gridSize[1] and grid[newX][newY] != 1 and (newX, newY) not in visited:
                    newDist = distance + 1
                    if newDist < distList[(newX, newY)][0]:
                        distList[(newX, newY)] = [newDist, node]
                    if (newDist, (newX, newY)) not in pQueue:
                        heapq.heappush(pQueue, (aStarHeuristic(newX, newY, endNode[0], endNode[1]) + newDist, (newX, newY)))        

                    if draw:
                        pointer.x, pointer.y = 10 + (boxSize + spacing)*newY, 10 + (boxSize + spacing)*newX
                        pygame.draw.rect(screen, colors["BLUE"], pointer)
                        pygame.draw.circle(screen, colors["GREEN"], (startNode[1] + 10, startNode[0] + 10), boxSize + 2)
                        pygame.draw.circle(screen, colors["GREEN"], (endNode[1] + 10, endNode[0] + 10), boxSize + 2)
                        pygame.event.get()   

                        if counter % speed == 0:
                            pygame.display.flip()       
        #Adds node to visited list to prevent getting stuck in a loop
        visited.add(node)    
        counter += 1

    return False, distList


def drawGrid(grid, boxSize, spacing):
    '''
    Draws a grid based on the conditions and the grid 
    '''
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 1:
                pygame.draw.rect(screen, colors["RED"], pointer)
            else:
                pygame.draw.rect(screen, colors["BLACK"], pointer)
            pygame.event.get()

            pointer.x += (boxSize + spacing)
        pointer.y += (boxSize + spacing)
        pointer.x = 10


def generateMaze(height, width):
    '''
    Uses DFS to build a maze
    Starts at the beginning, and branches out randomly until it meets a dead end
    It will then backtrack until it meets a branch with an available route, and repeats the branching
    '''
    #Used for DFS
    stack = collections.deque([(1, 1)])
    visited = set()
    directions = {(0, -2), (-2, 0), (0, 2), (2, 0)}
    path = set()

    while len(stack) > 0:
        #Removes the last element of the queue
        x, y = stack.pop()

        #Visits each direction and adds it to the queue if it is not visited
        neighbors = []
        for direction in directions:
            newX, newY = direction[0] + x, direction[1] + y
            if 0 <= newX < height and 0 <= newY < width and (newX, newY) not in visited:
                neighbors.append([(newX, newY), (newX-(direction[0]/2), newY-(direction[1]/2))])
                stack.appendleft((x, y))
        
        #Adds the maze route to the path list
        if len(neighbors) > 0:
            next = random.choice(neighbors)
            path.add(next[0])
            path.add(next[1])
            stack.append(next[0])
        visited.add((x, y))

    #Sets parts of the grid to either 1 or 0 depending on whether they are in the path list
    grid = [[0] * gridSize[1] for _ in range(gridSize[0])]
    for i in range(gridSize[0]):
        for j in range(gridSize[1]):
            if (i, j) not in path:
                grid[i][j] = 1
    grid[endNode[0]][endNode[1]] = 0
    grid[startNode[0]][startNode[1]] = 0
    
    return grid


#Setup
pygame.init()
screen = pygame.display.set_mode((1920, 1080), pygame.FULLSCREEN)
clock = pygame.time.Clock()

colors = {"GREEN": (0, 255, 0), "RED": (255, 0, 0), "BLUE": (0, 0, 255), "BLACK": (0, 0, 0)}
gridSize = (845, 1519)
boxSize = 1
spacing = 0

startNode = (1, 1)
endNode = (gridSize[0]-1, gridSize[1]-1)

speed = 100
heuristicWeight = 1.8

pointer = pygame.Rect(10, 10, boxSize, boxSize)

while 1:
    #Reset Screen
    screen.fill("white")
    pointer.x, pointer.y = 10, 10
    
    #Choses a random start node and end node that are at least 1/3 of the grid apart
    startNode = (random.randint(1, gridSize[0]), random.randint(1, gridSize[1]))
    endNode = startNode
    while endNode == startNode:
        endNode = (random.randint(1, gridSize[0]), random.randint(1, gridSize[1]))
    
    while diagonalDistance(startNode[0], startNode[1], endNode[0], endNode[1]) <= diagonalDistance(0, 0, gridSize[0], gridSize[1]) * 1/3:
        endNode = startNode
        while endNode == startNode:
            endNode = (random.randint(1, gridSize[0]), random.randint(1, gridSize[1]))
    
    #Generates a maze
    grid = generateMaze(gridSize[0], gridSize[1])

    #Regenerates until the maze is solveable
    pathSolved, distList = solvePath(grid)
    if not pathSolved:
        pygame.event.get()
        continue
    
    #Draws the grid
    drawGrid(grid, boxSize, spacing)
    pygame.draw.circle(screen, colors["GREEN"], (startNode[1] + 10, startNode[0] + 10), boxSize + 2)
    pygame.draw.circle(screen, colors["GREEN"], (endNode[1] + 10, endNode[0] + 10), boxSize + 2)
    pygame.display.flip()

    #Draws the A* algorithm
    solvePath(grid, True)

    #Draws the shortest path
    pointer.x, pointer.y = 10 + (boxSize + spacing)*(endNode[0]), 10 + (boxSize + spacing)*(endNode[1])
    curNode = endNode
    while 1:
        #Backtracks through the list of nodes, until the start node is met
        if curNode == startNode:
            break
        else:
            curNode = distList[curNode][1] 
            if type(curNode) == int:
                break
        pointer.x, pointer.y = 10 + (boxSize + spacing)*curNode[1], 10 + (boxSize + spacing)*curNode[0]
        pygame.draw.rect(screen, colors["GREEN"], pointer)
        pygame.event.get()
    pygame.display.flip()

    time.sleep(1)
