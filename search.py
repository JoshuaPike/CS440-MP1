# search.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Michael Abir (abir2@illinois.edu) on 08/28/2018

"""
This is the main entry point for MP1. You should only modify code
within this file -- the unrevised staff files will be used for all other
files and classes when code is run, so be careful to not modify anything else.
"""
# Search should return the path.
# The path should be a list of tuples in the form (row, col) that correspond
# to the positions of the path taken by your search algorithm.
# maze is a Maze object based on the maze from the file specified by input filename
# searchMethod is the search method specified by --method flag (bfs,dfs,astar,astar_multi,fast)
from queue import Queue
from queue import PriorityQueue
import copy

def search(maze, searchMethod):
    return {
        "bfs": bfs,
        "astar": astar,
        "astar_corner": astar_corner,
        "astar_multi": astar_multi,
        "fast": fast,
    }.get(searchMethod)(maze)

def bfs(maze):
    """
    Runs BFS for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    # Apply bfs to multiple dots
    q = Queue(maxsize = 0)
    visited = {}
    goals = maze.getObjectives()

    start = maze.getStart()
    q.put([start, start])

    toRet = []

    while not q.empty() and len(goals) != 0:
        cur = q.get()
        if cur[0] in visited.keys():
            continue
        
        visited[cur[0]] = cur[1]

        if cur[0] in goals:
            print("A goal has been found\n")
            goals.remove(cur[0])
            recurse = cur[0]
            
            while recurse != start:
                toRet.append(recurse)
                recurse = visited[recurse]
            toRet.append(start)
            toRet.reverse()
            visited.clear()

            while len(goals) != 0:
                pathAdd = []
                pathAdd = bfsHelper(maze, toRet[len(toRet) - 1], goals)
                toRet = toRet + pathAdd
            
            print(maze.isValidPath(toRet))
            return toRet

        for i in maze.getNeighbors(cur[0][0], cur[0][1]):
            if maze.isValidMove(i[0], i[1]):
                if i not in visited.keys():
                    q.put([i, cur[0]])

    return []


def astar(maze):
    """
    Runs A star for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    q = PriorityQueue(maxsize = 0)
    # Store tuple in prio queue by wrapping it with an orderable type (1, {'alpha': 1}) < (2, {'beta': 2})
    # q stores [ manhattan(node, goal) + dist from start, dist from start, node, previous node ]

    visited = {}
    # visited stores {node: (f(node), previous node)}
    goals = maze.getObjectives()

    start = maze.getStart()
    q.put([manhattan(start, goals[0]), 0, start, start])
    # print(manhattan(start, goals[0]), "\n")

    while not q.empty() and len(goals) != 0:
        # cur = [f(node), g(node), node, prev node]
        cur = q.get()
        # print("Current node: ", cur[2], "   Previous node: ", cur[3], "   Current f = ", cur[0], "   Dist to start = ", cur[1], "\n")
        

        visited[cur[2]] = (cur[0], cur[3])


        if cur[2] in goals:
            print("A goal has been found\n")
            recurse = cur[2]
            toRet = []

            while recurse != start:
                toRet.append(recurse)
                recurse = visited[recurse][1]
            toRet.append(start)
            toRet.reverse()

            print(maze.isValidPath(toRet))
            return toRet
        
        for i in maze.getNeighbors(cur[2][0], cur[2][1]):
            if maze.isValidMove(i[0], i[1]):
                f = manhattan(i, goals[0]) + cur[1] + 1
                # use < or <= ?
                if i not in visited.keys() or f < visited[i][0]:
                    q.put([f, cur[1] + 1, i, cur[2]])
                    visited[i] = (f, cur[2])

    return []

def astar_corner(maze):
    """
    Runs A star for part 2 of the assignment in the case where there are four corner objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
        """
    # Each state = [f(node), dist from start, node, current goals, prev node, prev goals]
    q = PriorityQueue(maxsize = 0)
    
    path = []
    visited = {}
    # Also store g in visited to break tie
    # visited stores {(node + goals): [current node's f(node), dist from start, prev node, previous goals]}

    startGoals = copy.deepcopy(maze.getObjectives())
    start = maze.getStart()
    firstState = [cornerManhattan(start, startGoals), 0, start, startGoals, (-1, -1), startGoals]
    q.put(firstState)

    while not q.empty():
        cur = q.get()
        # print("Cur node: ", cur[2], "    f: ", cur[0], "  g: ", cur[1], "\n")
        curState = (cur[2], tuple(cur[3]))

        visited[curState] = [cur[0], cur[1], cur[4], copy.deepcopy(cur[5])]

        # Check if current node is a goal
        if cur[2] in cur[3]:
            cur[3].remove(cur[2])
            visited[(cur[2], tuple(cur[3]))] = [cur[0], cur[1], cur[4], cur[5]]

            # Check if no goals left
            if len(cur[3]) == 0:
                recurseState = (cur[2], tuple(cur[3]))

                while recurseState[0] != (-1, -1):
                    path.append(recurseState[0])
                    recurseState = (visited[recurseState][2], tuple(visited[recurseState][3]))
                path.reverse()
                print(maze.isValidPath(path))
                return path
        
        # Add neighbors to queue
        # Check if deep copy of goals is needed
        for neigh in maze.getNeighbors(cur[2][0], cur[2][1]):
            if maze.isValidMove(neigh[0], neigh[1]):
                f = cornerManhattan(neigh, cur[3]) + cur[1] + 1
                if (neigh, tuple(cur[3])) not in visited.keys():
                    q.put([f, cur[1] + 1, neigh, copy.deepcopy(cur[3]), cur[2], cur[3]])
                    visited[(neigh, tuple(copy.deepcopy(cur[3])))] = [f, cur[1] + 1, cur[2], cur[3]]
                elif cur[1]+1 < visited[(neigh, tuple(cur[3]))][1]:
                    q.put([f, cur[1] + 1, neigh, copy.deepcopy(cur[3]), cur[2], cur[3]])
    return []

def astar_multi(maze):
    """
    Runs A star for part 3 of the assignment in the case where there are
    multiple objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    # Each state = [f(node), dist from start, node, current goals, prev node, prev goals]
    q = PriorityQueue(maxsize = 0)
    
    path = []
    visited = {}
    # Also store g in visited to break tie
    # visited stores {(node + goals): [current node's f(node), dist from start, prev node, previous goals]}

    startGoals = copy.deepcopy(maze.getObjectives())
    start = maze.getStart()
    firstState = [cornerManhattan(start, startGoals), 0, start, startGoals, (-1, -1), startGoals]
    q.put(firstState)

    while not q.empty():
        cur = q.get()
        # print("Cur node: ", cur[2], "    f: ", cur[0], "  g: ", cur[1], "\n")
        curState = (cur[2], tuple(cur[3]))

        visited[curState] = [cur[0], cur[1], cur[4], copy.deepcopy(cur[5])]

        # Check if current node is a goal
        if cur[2] in cur[3]:
            cur[3].remove(cur[2])
            visited[(cur[2], tuple(cur[3]))] = [cur[0], cur[1], cur[4], cur[5]]

            # Check if no goals left
            if len(cur[3]) == 0:
                recurseState = (cur[2], tuple(cur[3]))

                while recurseState[0] != (-1, -1):
                    path.append(recurseState[0])
                    recurseState = (visited[recurseState][2], tuple(visited[recurseState][3]))
                path.reverse()
                print(maze.isValidPath(path))
                return path
        
        # Add neighbors to queue
        # Check if deep copy of goals is needed
        for neigh in maze.getNeighbors(cur[2][0], cur[2][1]):
            if maze.isValidMove(neigh[0], neigh[1]):
                f = cornerManhattan(neigh, cur[3]) + cur[1] + 1
                if (neigh, tuple(cur[3])) not in visited.keys():
                    q.put([f, cur[1] + 1, neigh, copy.deepcopy(cur[3]), cur[2], cur[3]])
                    visited[(neigh, tuple(copy.deepcopy(cur[3])))] = [f, cur[1] + 1, cur[2], cur[3]]
                elif cur[1]+1 < visited[(neigh, tuple(cur[3]))][1]:
                    q.put([f, cur[1] + 1, neigh, copy.deepcopy(cur[3]), cur[2], cur[3]])
    return []


def fast(maze):
    """
    Runs suboptimal search algorithm for part 4.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    return []


# ---------------------- Helper functions below ----------------------------------------

def manhattan(node, goal):
    """
    Find the manhattan distance between two nodes

    @param node: The node you are currently at

    @param goal: The goal you are looking for

    @return manhattan distance: Manhattan distance between node and goal
    """
    dx = abs(node[0] - goal[0])
    dy = abs(node[1] - goal[1])
    return dx + dy

def cornerManhattan(node, goals):
    """
    Find the MINIMUM manhattan distance between a node and a set of goals. Specifically for astar_corner

    @param node: The node you are currently at

    @param goals: A list of all goals you are looking for

    @return manhattan distance: the minimum manhattan distance between a node and a goal
    """
    if len(goals) == 1:
        return manhattan(node, goals[0])
    
    smallest = manhattan(node, goals[0])
    smallestG = goals[0]
    for g in goals[1:]:
        man = manhattan(node, g)
        if man < smallest:
            smallest = man
            smallestG = g
    manDistGoals = []
    for g in goals:
        if g == smallestG:
            continue
        manDistGoals.append(manhattan(smallestG, g))
    return smallest + min(manDistGoals)

def bfsHelper(maze, start, goals):
    """
    A helper function so we can apply bfs to multiple nodes

    @param maze: The maze to execute the search on.

    @param start: The node we want to start at

    @param goals: A list of goals

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    q = Queue(maxsize = 0)
    visited = {}

    q.put([start, start])

    toRet = []

    while not q.empty() and len(goals) != 0:
        cur = q.get()
        if cur[0] in visited.keys():
            continue
        visited[cur[0]] = cur[1]

        if cur[0] in goals:
            # print("A goal has been found\n")
            recurse = cur[0]
            goals.remove(cur[0])

            while recurse != start:
                toRet.append(recurse)
                recurse = visited[recurse]
            toRet.reverse()
            return toRet

        for i in maze.getNeighbors(cur[0][0], cur[0][1]):
            if maze.isValidMove(i[0], i[1]):
                if i not in visited.keys():
                    q.put([i, cur[0]])

    return []

def mixer(keys):
    """
    A helper function to get the permutations of all pairs of values in a list

    @param keys: The items we want to get the permutations of. They will be tuples

    @return perms: The list of all possible pairs
    """
    # I'm assuming this implementation is correct
    perms = []
    if len(keys) == 0 or len(keys) == 1:
        return perms
    # print(len(keys))
    for i in range(0, len(keys)):
        for j in range(i + 1, len(keys)):
            # print("First Item: ", keys[i], "  Second Item: ", keys[j], "\n")
            perms.append([keys[i], keys[j]])
    # print(len(perms))
    return perms

# Store MST values between goals in dict bc we will use them a lot
# Edges of MST are the distance between different goals using A*
# When calculating these edge lengths DO NOT include the first (or last) path node