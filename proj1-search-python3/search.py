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

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    visited = []
    path = []
    stack =util.Stack()
    start = problem.getStartState()
    stack.push((start, path)) ##pushing the starting state in to the stack
    while not stack.isEmpty():
        current_state, path = stack.pop() ## poping the top of the stack and storing in the current state and path
        if current_state not in visited:  ## checking if current state is visited or not
            visited.append(current_state) ##if not visited I am making it visited and storing in visited list
            if problem.isGoalState(current_state):
                return path ## if we reached the goal I am returning the path
            for state, direction,cost in problem.getSuccessors(current_state): ##looping through the current state's successor nodes state,direction and cost
                stack.push((state, path + [direction]))   ## pushing the state and path in to the stack
    return path

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    visited = []
    path = []
    queue = util.Queue()
    start = problem.getStartState()
    queue.push((start,path)) ## pushing the start state in to the queue
    while not queue.isEmpty():
        current_state, path = queue.pop(); ## poping the element and storing it as current state
        if current_state not in visited: ## Checking if current state is visited or not
            visited.append(current_state) ## if not visited I am storing it in visited list
            if problem.isGoalState(current_state): ## if the current state  is the goal state I am returning the path
                return path
            for state, direction,cost in problem.getSuccessors(current_state): ## looping through the current state's successor state , direction and cost
                queue.push((state, path+[direction])) ## pushing the state and direction in to the queue
    return path


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    visited= list()
    path= list()
    pqueue = util.PriorityQueue()
    start = problem.getStartState()
    pqueue.push((start, path), 0) ## pushing the start element, path and cost in to the priority queue
    while not pqueue.isEmpty():
        best_cost_state, dir_from_start = pqueue.pop()
        ## poping the state and its corresponding path,cost and
        # storing the state in best_cost_state and the corresponding path from start state to the best_cost_state
        # and storing it in dir_from_start

        if problem.isGoalState(best_cost_state):
            return dir_from_start ## if the best_cost_state is the goal state I am returning the path

        if best_cost_state not in visited: ## Checking if the best_cost_state is visited
            visited.append(best_cost_state)## if it is not visited I am storing it in visited list
            for successor, parent_dir, stepCost in problem.getSuccessors(best_cost_state):
                ## glooping through the current state's successor's state , direction and cost
                updated_dir = dir_from_start + [parent_dir] ## updating the path
                cost = problem.getCostOfActions(updated_dir)## calculating the cost from the start to the updated path
                pqueue.push((successor, updated_dir), cost) ## pushing the successor state, updated path and the updated cost

    return []


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    visited = list()
    pqueue = util.PriorityQueue()
    start = problem.getStartState()
    pqueue.push((start,list(),0),0)
    best_cost_state,dir_from_start,total_cost_with_heuristic = pqueue.pop()
    visited.append((start,0))
    while not problem.isGoalState(best_cost_state): ## looping until the state found is not the goal state
        for successor, parent_dir, stepCost in problem.getSuccessors(best_cost_state):
            ## looping through the current best cost state's successor state, direction and cost
            updated_dir = dir_from_start+[parent_dir]
            ## updating the direction from start to present looped successor
            total_cost = problem.getCostOfActions(updated_dir)
            ## calculating the total cost for the updated direction
            flag = False
            for i in range(len(visited)):
                temp_state, temp_cost = visited[i]  # checks if node is at same position of successor
                if (successor == temp_state) and (total_cost >= temp_cost): ## if the node's total cost is greater than the total cost calculated
                    # until that node we donot consider that node further as in A* we choose the node with low backward cost
                    flag = True
            if not flag:
                total_cost = problem.getCostOfActions(dir_from_start + [parent_dir])
                ## calculating the total cost from start to that state
                pqueue.push((successor, dir_from_start + [parent_dir], total_cost),
                                total_cost + heuristic(successor, problem)) ##using the heuristic function to calculate the manhattan distance  and adding that with the total backward cost
                visited.append((successor, total_cost))## pushing the value set in to the priority queue
        best_cost_state, dir_from_start, total_cost_with_heuristic = pqueue.pop() ## poping the visited state with highest priority first
    return dir_from_start ## returning the path we got through A* search algo







# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
