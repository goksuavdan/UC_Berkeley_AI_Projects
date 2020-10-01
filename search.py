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
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    """
    Based on our DFS algorithm, we should find the item we need in the graph search.
    First of all, we should store the necessary nodes and whenever we reach the solution,
    we are going to do backtracking to show the solution for sure.
    """
    
    #They are going to have the nodes we are going to visit.
    explored_set = {}
    #This is for the parents and nodes.
    successor = {}
    stack = util.Stack()
    #Starting state insert to the stack
    stack.push( (problem.getStartState(), []) )
    
    while True:
        #Check the stack and give failure if necessary
        if stack.isEmpty():
            break
        #Node is removing from the stack
        node = stack.pop()
        #Storing the direction and the node.
        explored_set[node[0]] = node[1]
        #Checking the solution state.
        if problem.isGoalState(node[0]):
            solution_item = node[0]
            break
        else:
            solution = []
        for succ in problem.getSuccessors(node[0]):
            #Checking the successor is visited or not!!!
            if succ[0] not in explored_set.keys():
                successor[succ[0]] = node[0]
                #Basically, insert the succesor to top of the stack
                stack.push(succ)
            else:
                continue
                
    #Creating the empty solution list. Check this one later!
    solution = []
    while solution_item in successor.keys():
        #Inserting the direction for our solution output to the very top of the list.
        solution.insert(0, explored_set[solution_item])
        solution_item = successor[solution_item]
    return solution
    #util.raiseNotDefined()

def breadthFirstSearch(problem):
    """
    
    Search the shallowest nodes in the search tree first
    
    
    The comments are mostly be the same with the DFS method; I will just
    mention the necessary parts of the code.
    Thanks.
    
    """
    explored_set = {}
    #Using Queue instead of Stack in this uninformed search method.
    queue = util.Queue()
    successor = {}
    queue.push( (problem.getStartState(), []) )
    while True:
        if queue.isEmpty():
            break
        node = queue.pop()
        explored_set[node[0]] = node[1]
        if problem.isGoalState(node[0]):
            solution_item = node[0]
            break
        for succ in problem.getSuccessors(node[0]):
            #We are going to check the parents and the expanded nodes at the same time.
            #This is the difference of BFS comparing to DFS.
            if succ[0] not in explored_set.keys():
            #Checking the explored set, otherwise that can continue forever.
                if succ[0] not in successor.keys():
                    successor[succ[0]] = node[0]
                    queue.push(succ)
    solution = []
    while(solution_item in successor.keys()):
        solution.insert(0, explored_set[solution_item])
        solution_item = successor[solution_item]
    return solution
    #util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    explored_set = {}
    #Using Priority Queue instead of Stack or Queue in this uninformed search method.
    #But heuristic is not include this method.
    queueuni = util.PriorityQueue()
    successor = {}
    cost = {}
    #Update the queue with StartState
    queueuni.update( (problem.getStartState(), 'None', 0), [] )
    while True:
        if queueuni.isEmpty():
            break
        node = queueuni.pop()
        explored_set[node[0]] = node[1]
        if problem.isGoalState(node[0]):
            solution_item = node[0]
            break
        for succ in problem.getSuccessors(node[0]):
            #Estimation of the new cost for new childs.
            if succ[0] not in explored_set.keys():
                priority = node[2] + succ[2]
                #Check the cost comparison before execution.
                if succ[0] in cost.keys():
                    #Checking the cost of the priority.
                    if priority > cost[succ[0]]:
                        continue
                #Chaning the cost function.
                queueuni.update((succ[0], succ[1], node[2] + succ[2]), priority)
                cost[succ[0]] = priority
                successor[succ[0]] = node[0]
                
    solution = []
    while(solution_item in successor.keys()):
        solution.insert(0, explored_set[solution_item])
        solution_item = successor[solution_item]
    return solution
    #util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    explored_set = {}
    solution = []
    queueA = util.PriorityQueue()
    successor = {}
    cost = {}
    start = problem.getStartState()
    queueA.update((start, 'None', 0), 0)
    while True:
        if queueA.isEmpty():
            break
        node = queueA.pop()
        explored_set[node[0]] = node[1]
        if problem.isGoalState(node[0]):
            solution_item = node[0]
            break
        #There is a little difference comparing to Uniformed Cost Search,
        #We are also looking for the heuristic. So, our new method is
        #Informed Cost Search and more optimal for sure.
        for succ in problem.getSuccessors(node[0]):
            if succ[0] not in explored_set.keys():
                priority = node[2] + succ[2] + heuristic(succ[0], problem)
                #Comparing the previous method, we should account heuristis as well.
                if succ[0] in cost.keys():
                    if priority > cost[succ[0]]:
                    #Check the new cost first
                        continue
                #If the cost is less than the previous estimated cost,
                #we are going to change the cost with updated one
                #as well as the parents
                queueA.push((succ[0], succ[1], node[2] + succ[2]), priority)
                cost[succ[0]] = priority
                successor[succ[0]] = node[0]

    solution = []
    while(solution_item in successor.keys()):
        solution.insert(0, explored_set[solution_item])
        solution_item = successor[solution_item]
    return solution
    #util.raiseNotDefined()

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
