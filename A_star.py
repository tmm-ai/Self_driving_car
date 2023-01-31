import math

"""""
Code mostly taken FROM ALGOEXPERT.IO, to execute the A* algorithm. 
"""""
class Node:
    def __init__(self, row, col, value):
        self.id = str(row) + "-" + str(col)
        self.row = row
        self.col = col
        self.value = value
        self.distanceFromStart = float('inf') # g score  ***
        self.estimatedDistanceToEnd = float('inf') # f score ***
        self.cameFrom = None  # we do not know this until we first move


def aStarAlgorithm(startRow, startCol, endRow, endCol, graph):
    """""
    This function is called from the navigation file and utilizes all helper functions in this module. It takes in 
    the starting location, destination and a 2D matrix laying out where all the obstacles and padding are located
    and returns the shortest path. 
    Inputs: starting position, destination, 2D array
    Output: 2D array of 2D coordinates where each index is the next step along the optimal path. 
    
    """""
    nodes = initializeNodes(graph)

    startNode = nodes[startRow][startCol]
    endNode = nodes[endRow][endCol]

    startNode.distanceFromStart = 0
    # calculating f score
    startNode.estimatedDistanceToEnd = calculate_Distance(startNode, endNode)

    nodesToVisit = MinHeap([startNode])

    while not nodesToVisit.isEmpty():
        # this gets the node with the smallest f score and removes it
        currentMinDistanceNode = nodesToVisit.remove()

        if currentMinDistanceNode == endNode:
            break # this means we reached the goal
        # if not at goal, get all neighbors
        neighbors = getNeighboringNodes(currentMinDistanceNode, nodes)
        for neighbor in neighbors:
            # this checks if a neighbor is an obstacle, if so, ignore it.
            if neighbor.value != 0:
                continue
            # this is distance from start node to neighbor
            tentativeDistanceToNeighbor = currentMinDistanceNode.distanceFromStart + 1

            # this means that the neighbor already has a shorter or equal distance to end,
            # as the current node, so we will not update it, we will skip over it.
            if tentativeDistanceToNeighbor >= neighbor.distanceFromStart:
                continue

            neighbor.cameFrom = currentMinDistanceNode
            neighbor.distanceFromStart = tentativeDistanceToNeighbor
            # below is essentially f score = g score + h score
            neighbor.estimatedDistanceToEnd = tentativeDistanceToNeighbor + calculate_Distance(neighbor, endNode)

            # adding nodes to node to visit if not there already
            if not nodesToVisit.containsNode(neighbor):
                nodesToVisit.insert(neighbor)
            # updating node with shorter distance if it was already in there.
            else:
                nodesToVisit.update(neighbor)

    return reconstructPath(endNode)


def initializeNodes(graph):
    """""
    This function takes in a 2D array of integers and returns a 2D array of node objects. 
    """""
    nodes = [] # we recreate graph, but instead of using 1,0, we use nodes.

    for i, row in enumerate(graph):
        nodes.append([])  # append empty list/array
        for j, value in enumerate(row): # going over rows
            nodes[i].append(Node(i, j, value)) # i is row, j is col, value is 0/1 based on graph

    return nodes  # returns 2D array of node objects


def calculate_Distance(currentNode, endNode):
    """""
    Calculating Euclidean distance
    """""
    currentRow = currentNode.row
    currentCol = currentNode.col
    endRow = endNode.row
    endCol = endNode.col

    return math.sqrt((currentRow - endRow)**2 + (currentCol - endCol)**2)


def getNeighboringNodes(node, nodes):
    """""
    Creates a list of neighbor nodes for each node that is sent. 
    """""
    neighbors = []

    numRows = len(nodes)
    numCols = len(nodes[0])

    row = node.row
    col = node.col

    if row < numRows + 1 and col < numCols + 1:  # upper left neighbor
        neighbors.append(nodes[row - 1][col - 1])

    if row < numRows - 1 and col < numCols + 1:  # bottom left neighbor
        neighbors.append(nodes[row + 1][col - 1])

    if row < numRows + 1 and col < numCols -1:  # upper right neighbor
        neighbors.append(nodes[row - 1][col +1])

    if row < numRows - 1 and col < numCols -1:  # bottom right neighbor
        neighbors.append(nodes[row + 1][col +1])

    if row < numRows - 1:  # DOWN neighbor
        neighbors.append(nodes[row + 1][col])

    if row > 0:  # UP neighbor
        neighbors.append(nodes[row - 1][col])

    if col < numCols - 1:  # RIGHT neighbor
        neighbors.append(nodes[row][col + 1])

    if col > 0:  # LEFT neighbor
        neighbors.append(nodes[row][col - 1])

    return neighbors


def reconstructPath(endNode):
    """""
    Converts a 2D array of nodes to a 2D array of integers which is the step by step movement from the starting
    position to the end position.
    """""
    if not endNode.cameFrom:
        return []   # this means we did not find a valid path

    currentNode = endNode
    path = []

    # this creates a list of nodes / cells that provides the path to end
    while currentNode is not None:
        path.append([currentNode.row, currentNode.col])
        currentNode = currentNode.cameFrom

    # reversing path to go from start to end, since we start from endNode
    return path[::-1]

class MinHeap:
    def __init__(self, array):
        # Holds the positions of all nodes in the heap. Need to track where nodes are in
        # heap so that when we update a node, we have the nodes in the correct place
        self.nodePositionInHeap = {node.id: idx for idx, node in enumerate(array)}
        self.heap = self.buildHeap(array)

    def isEmpty(self):
        return len(self.heap) == 0

    def buildHeap(self, array):
        firstParentIdx = (len(array) - 2) // 2
        for currentIdx in reversed(range(firstParentIdx + 1)):
            self.siftDown(currentIdx, len(array) - 1, array)
        return array

    # we are sorting nodes by their f score (estimated distance to end)
    def siftDown(self, currentIdx, endIdx, heap):
        childOneIdx = currentIdx * 2 + 1
        while childOneIdx <= endIdx:
            childTwoIdx = currentIdx * 2 + 2 if currentIdx * 2 + 2 <= endIdx else -1
            if (childTwoIdx != 1 and heap[childTwoIdx].estimatedDistanceToEnd < heap[
                childOneIdx].estimatedDistanceToEnd):
                idxToSwap = childTwoIdx
            else:
                idxToSwap = childOneIdx
            if heap[idxToSwap].estimatedDistanceToEnd < heap[currentIdx].estimatedDistanceToEnd:
                self.swap(currentIdx, idxToSwap, heap)
                currentIdx = idxToSwap
                childOneIdx = currentIdx * 2 + 1
            else:
                return

    def siftUp(self, currentIdx, heap):
        parentIdx = (currentIdx - 1) // 2
        while currentIdx > 0 and heap[currentIdx].estimatedDistanceToEnd < heap[parentIdx].estimatedDistanceToEnd:
            self.swap(currentIdx, parentIdx, heap)
            currentIdx = parentIdx
            parentIdx = (currentIdx - 1) // 2

    def remove(self):
        if self.isEmpty():
            return
        # when we remove nodes, we remove it from the heap position too.
        self.swap(0, len(self.heap) - 1, self.heap)
        node = self.heap.pop()
        del self.nodePositionInHeap[node.id]
        self.siftDown(0, len(self.heap) - 1, self.heap)
        return node

    def insert(self, node):
        self.heap.append(node)
        self.nodePositionInHeap[node.id] = len(self.heap) - 1
        self.siftUp(len(self.heap) - 1, self.heap)

    def swap(self, i, j, heap):
        self.nodePositionInHeap[heap[i].id] = j
        self.nodePositionInHeap[heap[j].id] = i
        heap[i], heap[j] = heap[j], heap[i]

    # tells us if node is in heap
    def containsNode(self, node):
        return node.id in self.nodePositionInHeap

    # updates position of node when f or g score is updated.
    def update(self, node):
        self.siftUp(self.nodePositionInHeap[node.id], self.heap)
