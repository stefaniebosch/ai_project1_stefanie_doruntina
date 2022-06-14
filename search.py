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
    """

    #print("Start:", problem.getStartState())
    #print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    #print("Start's successors:", problem.getSuccessors(problem.getStartState()))

	#use a stack because dfs uses a LIFO queue
    from util import Stack
    frontier = Stack()
    already_explored = []
    actions_to_goal = []

    #first, test to check if initial state is already the goal stage; if yes, no path is taken, return empty array
    if problem.isGoalState(problem.getStartState()):
        return []

    #push initial state and empty action list in frontier to start it off
    frontier.push((problem.getStartState(), actions_to_goal))

    #keep going through all possibilites until frontier is empty (not counting repeated states)
    while not frontier.isEmpty():

        #since we're using LIFO queue, last node in is the next one to be explored, so we explore depth-first
        next_node_to_explore, actions_to_goal = frontier.pop()
        print("next node to explore: " + str(next_node_to_explore))
        #actions_to_goal.append(next_node_to_explore)

        #since we are not expanding on already visited states
        if next_node_to_explore not in already_explored:
            already_explored.append(next_node_to_explore)

            #once we hit the goal, return the path to the goal
            if problem.isGoalState(next_node_to_explore):
                print("found goal state!: " + str(actions_to_goal))
                return actions_to_goal

            #find successors of node we are currently exploring
            for succesor_x in problem.getSuccessors(next_node_to_explore):
                updated_actions_to_goal = actions_to_goal + [succesor_x[1]]
                frontier.push((succesor_x[0], updated_actions_to_goal))

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    #use a queue because dfs uses a FIFO queue
    from util import Queue
    frontier = Queue()
    already_explored = []
    actions_to_goal = []

    #first, test to check if initial state is already the goal stage; if yes, no path is taken, return empty array
    if problem.isGoalState(problem.getStartState()):
        return []

    #push initial state and empty action list in frontier to start it off
    frontier.push((problem.getStartState(), actions_to_goal))

    #keep going through all possibilites until frontier is empty (not counting repeated states)
    while not frontier.isEmpty():

        #since we're using FIFO queue, most recent node in is the next one to be explored, so we explore breadth-first
        next_node_to_explore, actions_to_goal = frontier.pop()
        print("next node to explore: " + str(next_node_to_explore))
        #actions_to_goal.append(next_node_to_explore)

        #since we are not expanding on already visited states
        if next_node_to_explore not in already_explored:
            already_explored.append(next_node_to_explore)

            #once we hit the goal, return the path to the goal
            if problem.isGoalState(next_node_to_explore):
                print("found goal state!: " + str(actions_to_goal))
                return actions_to_goal

            #find successors of node we are currently exploring
            for succesor_x in problem.getSuccessors(next_node_to_explore):
                updated_actions_to_goal = actions_to_goal + [succesor_x[1]]
                frontier.push((succesor_x[0], updated_actions_to_goal))

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    #use a queue because dfs uses a FIFO queue
    from util import PriorityQueue
    frontier = PriorityQueue()
    already_explored = []
    actions_to_goal = []

    #first, test to check if initial state is already the goal stage; if yes, no path is taken, return empty array
    if problem.isGoalState(problem.getStartState()):
        return []

    #push initial state, empty action list, current node cost, and priority in frontier to start it off
    frontier.push((problem.getStartState(), actions_to_goal, 0), 0)

    #keep going through all possibilites until frontier is empty (not counting repeated states)
    while not frontier.isEmpty():

        #since we're using FIFO queue, most recent node in is the next one to be explored, so we explore breadth-first
        next_node_to_explore, actions_to_goal, current_cost = frontier.pop()
        print("next node to explore: " + str(next_node_to_explore))
        #actions_to_goal.append(next_node_to_explore)

        #since we are not expanding on already visited states
        if next_node_to_explore not in already_explored:
            already_explored.append(next_node_to_explore)

            #once we hit the goal, return the path to the goal
            if problem.isGoalState(next_node_to_explore):
                print("found goal state!: " + str(actions_to_goal))
                return actions_to_goal

            #find successors of node we are currently exploring
            for succesor_x in problem.getSuccessors(next_node_to_explore):
                updated_actions_to_goal = actions_to_goal + [succesor_x[1]]
                new_cost = current_cost + succesor_x[2]
                frontier.push((succesor_x[0], updated_actions_to_goal, new_cost), new_cost)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
