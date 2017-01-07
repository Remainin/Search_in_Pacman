# coding=utf-8
# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        util.raiseNotDefined()


class Node():#用于之后各结点的处理
    """
    A container storing the current state of a node, the list 
    of  directions that need to be followed from the start state to
    get to the current state and the specific problem in which the
    node will be used.
    """
    def __init__(self, state, path, cost=0, heuristic=0, problem=None):
	self.state = state
	self.path = path
	self.cost = cost
	self.heuristic = heuristic
	self.problem = problem

    def __str__(self):
	string = "Current State: "
	string += __str__(self.state)
	string += "\n"
	string == "Path: " + self.path + "\n"
	return string

    def getSuccessors(self, heuristicFunction=None):
	children = []	
	for successor in self.problem.getSuccessors(self.state):
	    state = successor[0]
	    path = list(self.path)
	    path.append(successor[1])
	    cost = self.cost + successor[2]
	    if heuristicFunction:
		heuristic = heuristicFunction(state, self.problem)
	    else:
		heuristic = 0
	    node = Node(state, path, cost, heuristic, self.problem)
	    children.append(node)
	return children


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s,s,w,s,w,w,s,w]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first

    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())

    """
    #此部分按照老师所给出的伪代码进行设计
    closed = set()         #close表用于保存已经遍历过的点
    fringe = util.Stack()  #此部分相当于老师上课所讲的open表，用栈来表示
    startNode = Node(problem.getStartState(), [], 0, 0, problem)
    fringe.push(startNode)  #首先将开始结点放入open表，即压栈fringe  

    while True:
	if fringe.isEmpty():   #若栈空，则说明已经遍历了所有的结点
	    return False
	node = fringe.pop()    #pop出一个结点进行判断
        #print node.state
	if problem.isGoalState(node.state):
            #print node.path
	    return node.path   #若该结点为目标结点，则结束
	if node.state not in closed: #否则，首先将此结点放入close表中，标志为
	    closed.add(node.state)   #已经遍历过，再将此结点的孩子结点压入栈，
	    for childNode in node.getSuccessors():#进入下一层循环
	        fringe.push(childNode)

def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    """
    closed = set()         #close表用于保存已经遍历过的点
    fringe = util.Queue()  #此部分相当于老师上课所讲的open表，用队列来表示

    startNode = Node(problem.getStartState(), [], 0, 0, problem)
    fringe.push(startNode)  #首先将开始结点放入open表，即入队fringe

    while True:
  	if fringe.isEmpty():   #若队列为空，则说明已经遍历了所有的结点
	    return False
	node = fringe.pop()   #从队列中出队出一个结点进行判断
        #print node.state
	if problem.isGoalState(node.state):
            #print node.path
	    return node.path  #若该结点为目标结点，则结束
	if node.state not in closed: #否则，首先将此结点放入close表中，标志为
	    closed.add(node.state)   #已经遍历过，再将此结点的孩子结点入队列，
	    for childNode in node.getSuccessors():#进入下一层循环
		fringe.push(childNode)

def uniformCostSearch(problem):
    "Search the node of least total cost first. "

    closed = set()         #close表用于保存已经遍历过的点
    fringe = util.PriorityQueue()  
                    #此部分相当于老师上课所讲的open表，用优先队列来表示

    startNode = Node(problem.getStartState(), [], 0, 0, problem)
    fringe.push(startNode, startNode.cost)
    while True:           #首先将开始结点和其对应的代价值cost，入队fringe
	if fringe.isEmpty():  #若队列为空，则说明已经遍历了所有的结点
	    return False
	node = fringe.pop()   #从队列中出队出一个结点进行判断
        #print node.state
	if problem.isGoalState(node.state):
            #print node.path
	    return node.path  #若该结点为目标结点，则结束
	if node.state not in closed: #否则，首先将此结点放入close表中，标志为
	    closed.add(node.state)   #已经遍历过将此结点的孩子结点以及其对应cost
	    for childNode in node.getSuccessors():#入队，进入下一层循环
		fringe.push(childNode, childNode.cost)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    "Search the node that has the lowest combined cost and heuristic first."

    closed = set()         #close表用于保存已经遍历过的点
    fringe = util.PriorityQueue()
                #此部分相当于老师上课所讲的open表，用优先队列来表示
    startNode = Node(problem.getStartState(), [], 0, 0, problem)
    fringe.push(startNode, startNode.cost + startNode.heuristic)

    while True:           #首先将开始结点和其对应的cost与h(x)即曼哈顿距离的和，
	if fringe.isEmpty():  #入队fringe
	    return False      #若队列为空，则说明已经遍历了所有的结点
	node = fringe.pop()   #从队列中出队出一个结点进行判断
        #print node.state,node.heuristic
	if problem.isGoalState(node.state):
            #print node.path
	    return node.path
	if node.state not in closed: #否则，首先将此结点放入close表中，标志为已经
	    closed.add(node.state)   #遍历过将此结点的孩子结点以及其对应实际代价
	    for childNode in node.getSuccessors(heuristic):#入队，进入下一层循环
		fringe.push(childNode, childNode.cost + childNode.heuristic)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
