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
    "*** YOUR CODE HERE ***"
    from util import Stack
    openList = Stack() 
    # DFS use stack 
    closedList = []
    startPoint = problem.getStartState()
    if not problem.isGoalState(startPoint):
        # check if startPoint is the goalstate, if not, push the startpoint into openlist. Else, stop searching. 
        openList.push((startPoint, []))
    else:
        return 'STOP'
    while openList:
        # when openlist is not empty, pop off the last-in item and label it with node and path to it.
        cur,path = openList.pop()
        if problem.isGoalState(cur):
            return path
        if cur not in closedList:
        # if current pop-off item not in closed list, add it into.
            closedList.append(cur)
            succs = problem.getSuccessors(cur)
                
            for succ_info in succs:
            # check if the successor node in the closed list already, if not, update the path into it 
            # and push node and path into open list. 
                node = succ_info[0]
                action = succ_info[1]
                newCost = succ_info[2]
                if node not in closedList:
                    totalPath = path + [action]
                    openList.push((node,totalPath))

    util.raiseNotDefined()



def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    from util import Queue
    openList = Queue()
    # BFS use queue 
    closedList = []
    startPoint = problem.getStartState()
    if not problem.isGoalState(startPoint):
        # check if startPoint is the goalstate, if not, push the startpoint into openlist. Else, stop searching. 
        openList.push((startPoint, []))
    else:
        return 'STOP'
    while openList:
        # when openlist is not empty, pop off the first-in item and label it with node and path to it.
        cur,path = openList.pop()
        if problem.isGoalState(cur):
            return path
        if cur not in closedList:
            # if current pop-off item not in closed list, add it into.
            closedList.append(cur)
            succs = problem.getSuccessors(cur)

            for succ_info in succs:
                # check if the successor node in the closed list already, if not, update the path into it 
                # and push node and path into open list. 
                node = succ_info[0]
                action = succ_info[1]
                newCost = succ_info[2]
                if node not in closedList:
                    totalPath = path + [action]
                    openList.push((node,totalPath))



    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue
    openList = PriorityQueue()
    # ucs uses PriorityQueue
    closedList = []
    startPoint = problem.getStartState()
    if not problem.isGoalState(startPoint):
        # check if startPoint is the goalstate, if not, push the startpoint into openlist. Else, stop searching. 
        openList.push((startPoint,[],0),0)
    else:
        return 'STOP'
    
    while openList:
        # when openlist is not empty, pop off item and label it with node and path to it.
        cur_node,path,cost = openList.pop() 
         
        if problem.isGoalState(cur_node):
            return path
        if cur_node not in closedList:
            # if current pop-off item not in closed list, add it into.
            closedList.append(cur_node)
            succs = problem.getSuccessors(cur_node)
            

            for succ_info in succs:
                # check if the successor node in the closed list already, if not, update the path, cost into it 
                # and push node and path,totalcost into open list. 
                node = succ_info[0]
                action = succ_info[1]
                newCost = succ_info[2]
                if node not in closedList:
                    totalPath = path + [action]
                    totalCost = cost + newCost
                    openList.push((node,totalPath,totalCost),totalCost)


    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue
    openList = PriorityQueue()
    # astar uses PriorityQueue
    closedList = []
    startPoint = problem.getStartState()
    if not problem.isGoalState(startPoint):
        # check if startPoint is the goalstate, if not, push the startpoint into openlist. Else, stop searching. 
        openList.push((startPoint,[],0),0)
    else:
        return 'STOP'
    
    while openList:
        # when openlist is not empty, pop off item and label it with node and path to it.
        cur_node,path,cost = openList.pop() 
        if problem.isGoalState(cur_node):
            return path
        if cur_node not in closedList:
            # if current pop-off item not in closed list, add it into.
            closedList.append(cur_node)
            succs = problem.getSuccessors(cur_node)
            

            for succ_info in succs:
                # check if the successor node in the closed list already, if not, update the path, cost into it 
                # and push node and path,total cost with and without heuristic cost into open list. 
                node = succ_info[0]
                action = succ_info[1]
                newCost = succ_info[2]
                if node not in closedList:
                    totalPath = path + [action]
                    totalCost = cost + newCost + heuristic(node,problem)
                    openList.push((node,totalPath,(cost + newCost)),totalCost)

    util.raiseNotDefined()


# Abbreviations

bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
