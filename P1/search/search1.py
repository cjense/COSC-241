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
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """Search the deepest nodes in the search tree first."""

    directions = []
    node = (problem.getStartState(), directions)
    frontier = util.Stack()
    explored = []

    # check if initial state is a goal
    if problem.isGoalState(node[0]):
        return directions

    frontier.push(node)

    while not frontier.isEmpty():
        node = frontier.pop()
        if problem.isGoalState(node[0]):
            return node[1]

        if node[0] not in explored:
            explored.append(node[0])
            for child in problem.getSuccessors(node[0]):
                state, action, cost = child

                actions = list(node[1])
                actions.append(action)
                child1 = [state, actions]
                frontier.push(child1)

    # https://www.geeksforgeeks.org/stack-in-python/
    # https://www.jquery-az.com/4-demos-python-if-not-and-not-in-operator/
    # https://towardsdatascience.com/append-in-python-41c37453400?gi=be89e787f163#:~:text=The%20append()%20method%20in,the%20list%20increases%20by%20one.
    # https://stackoverflow.com/questions/11902107/list-vs-arraylist-vs-dictionary-vs-hashtable-vs-stack-vs-queue
    # https://www.geeksforgeeks.org/a-search-algorithm/



def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""

    directions = []
    node = (problem.getStartState(), directions)
    frontier = util.Queue()
    explored = []

    # check if initial state is a goal
    if problem.isGoalState(node[0]):
        return directions

    frontier.push(node)

    while not frontier.isEmpty():
        node = frontier.pop()
        if problem.isGoalState(node[0]):
            return node[1]

        if node[0] not in explored:
            explored.append(node[0])
            for child in problem.getSuccessors(node[0]):
                state, action, cost = child

                actions = list(node[1])
                actions.append(action)
                child1 = [state, actions]
                frontier.push(child1)

def uniformCostSearch(problem):
    """Search the node of least total cost first."""

    directions = []
    cost = 0
    node = (problem.getStartState(), directions)
    frontier = util.PriorityQueue()
    explored = []

    # check if initial state is a goal
    if problem.isGoalState(node[0]):
        return node[1]

    frontier.push(node, cost)

    while not frontier.isEmpty():
        node1 = frontier.pop()
        print(node1)
        if problem.isGoalState(node1[0][0]):
            return node1[0][1]

        if node1[0][0] not in explored:
            explored.append(node1[0][0])
            for child in problem.getSuccessors(node1[0][0]):
                state, action, cost1 = child

                actions = list(node[0][1])
                actions.append(action)

                cost1 += node1[1]
                nodeInfo = (state, actions)
                frontier.push(nodeInfo, cost1)


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""

    directions = []
    node = (problem.getStartState(), directions)
    frontier = util.PriorityQueue()
    explored = []

    # check if initial state is a goal
    if problem.isGoalState(node[0]):
        return directions

    frontier.push(node, 0 + heuristic(node[0], problem))

    while not frontier.isEmpty():
        node = frontier.pop()
        if problem.isGoalState(node[0]):
            return node[1]

        if node[0] not in explored:
            explored.append(node[0])
            for child in problem.getSuccessors(node[0]):
                state, action, cost = child

                actions = list(node[1])
                actions.append(action)
                child1 = [state, actions]
                frontier.push(child1, cost + heuristic(state, problem))
                # parent cost plus own cost


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
