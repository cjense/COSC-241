# multiAgents.py
# --------------
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

# Project 2 completed in Fall 2021 by Claire Jensen and Maryam Abuissa for
# Professor Scott Alfeld's Artificial Intelligence course

from util import manhattanDistance
from game import Directions
import random, util

from game import Agent

class ReflexAgent(Agent):
    """
      A reflex agent chooses an action at each choice point by examining
      its alternatives via a state evaluation function.

      The code below is provided as a guide.  You are welcome to change
      it in any way you see fit, so long as you don't touch our method
      headers.
    """


    def getAction(self, gameState):
        """
        You do not need to change this method, but you're welcome to.

        getAction chooses among the best options according to the evaluation function.

        Just like in the previous project, getAction takes a GameState and returns
        some Directions.X for some X in the set {North, South, West, East, Stop}
        """
        # Collect legal moves and successor states
        legalMoves = gameState.getLegalActions()

        # Choose one of the best actions
        scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
        bestScore = max(scores)
        bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
        chosenIndex = random.choice(bestIndices)  # Pick randomly among the best

        return legalMoves[chosenIndex]

    def evaluationFunction(self, currentGameState, action):
        """
        Design a better evaluation function here.

        The evaluation function takes in the current and proposed successor
        GameStates (pacman.py) and returns a number, where higher numbers are better.

        The code below extracts some useful information from the state, like the
        remaining food (newFood) and Pacman position after moving (newPos).
        newScaredTimes holds the number of moves that each ghost will remain
        scared because of Pacman having eaten a power pellet.

        Print out these variables to see what you're getting, then combine them
        to create a masterful evaluation function.
        """
        successorGameState = currentGameState.generatePacmanSuccessor(action)
        newPos = successorGameState.getPacmanPosition()
        newFood = successorGameState.getFood()
        newGhostStates = successorGameState.getGhostStates()
        newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

        width = newFood.packBits()[0]
        height = newFood.packBits()[1]
        walls = currentGameState.getWalls()

        # If the position we are looking at is in a wall,
        # make this very undesirable
        if walls[newPos[0]][newPos[1]]:
            solution = -10000
            return solution

        # If the position is where Pacman is, make this
        # very undesirable
        if newPos == currentGameState.getPacmanPosition():
            solution = -111111111111
            return solution

        # Find Manhattan distance between ghost's next move
        # and our next position
        numGhosts = 0
        ghostDist = width + height
        for pos in successorGameState.getGhostPositions():
            distance = util.manhattanDistance(pos, newPos)
            if distance < ghostDist:
                ghostDist = distance
            numGhosts += 1

        foodDist = closestFood(newPos, newFood, width, height)
        foodTotal = totalFood(newFood, width, height)

        minScaredTime = min(newScaredTimes)

        # When there are any ghosts who have scared time left, pursue them
        # until there's only one scared amount left, then go back to running away
        if minScaredTime > 1:
            solution = 3 * (width + height) - 15 * ghostDist - foodDist - foodTotal
        elif ghostDist > 3:
            solution = 3 * (width + height) + ghostDist - 2 * foodDist - (width + height) * foodTotal
        else:
            solution = (width + height) * ghostDist - foodDist - (width + height) * foodTotal

        return solution


def closestFood(position, food, w, h):
    positionOfClosestFood = (-1, -1)
    minDist = w + h
    hasFood = False

    # Compute minimum distance between Pacman and
    # every food in range w, h
    for i in range(w):
        for j in range(h):
            if food[i][j]:
                hasFood = True
                dist = util.manhattanDistance(position, (i, j))
                if dist < minDist:
                    minDist = dist
                    positionOfClosestFood = (i, j)

    if not hasFood:
        return 0

    return util.manhattanDistance(position, positionOfClosestFood)


def totalFood(food, w, h):
    runningTotal = 0

    # Check all positions in range w, h
    # and add to running food total
    for i in range(w):
        for j in range(h):
            if food[i][j]:
                runningTotal += 1

    return runningTotal

def scoreEvaluationFunction(currentGameState):
    """
      This default evaluation function just returns the score of the state.
      The score is the same one displayed in the Pacman GUI.

      This evaluation function is meant for use with adversarial search agents
      (not reflex agents).
    """
    return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
    """
      This class provides some common elements to all of your
      multi-agent searchers.  Any methods defined here will be available
      to the MinimaxPacmanAgent, AlphaBetaPacmanAgent & ExpectimaxPacmanAgent.

      You *do not* need to make any changes here, but you can if you want to
      add functionality to all your adversarial search agents.  Please do not
      remove anything, however.

      Note: this is an abstract class: one that should not be instantiated.  It's
      only partially specified, and designed to be extended.  Agent (game.py)
      is another abstract class.
    """

    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0  # Pacman is always agent index 0
        self.evaluationFunction = util.lookup(evalFn, globals())
        self.depth = int(depth)

class MinimaxAgent(MultiAgentSearchAgent):
    """
      Your minimax agent (question 2)
    """

    def getAction(self, gameState):
        """
          Returns the minimax action from the current gameState using self.depth
          and self.evaluationFunction.

          Here are some method calls that might be useful when implementing minimax.

          gameState.getLegalActions(agentIndex):
            Returns a list of legal actions for an agent
            agentIndex=0 means Pacman, ghosts are >= 1

          gameState.generateSuccessor(agentIndex, action):
            Returns the successor game state after an agent takes an action

          gameState.getNumAgents():
            Returns the total number of agents in the game
        """

        utilities = []
        for action in gameState.getLegalActions(0):
            newState = gameState.generateSuccessor(0, action)
            utilities.append(self.minimax(newState, 1, 1))

        max = utilities[0]
        actionIndex = 0

        for i in range(len(utilities)):
            if utilities[i] > max:
                max = utilities[i]
                actionIndex = i

        return gameState.getLegalActions(0)[actionIndex]

    def minimax(self, gameState, agentIndex, currDepth):
        newAgentIndex = (agentIndex + 1) % gameState.getNumAgents()

        if agentIndex == 0:
            currDepth += 1

        if gameState.isWin() or gameState.isLose():
            return self.evaluationFunction(gameState)
        if currDepth > self.depth:
            return self.evaluationFunction(gameState)

        values = []
        for action in gameState.getLegalActions(agentIndex):
            state = gameState.generateSuccessor(agentIndex, action)
            values.append(self.minimax(state, newAgentIndex, currDepth))

        if agentIndex == 0:
            return max(values)
        elif agentIndex > 0:
            return min(values)

class AlphaBetaAgent(MultiAgentSearchAgent):
    """
      Your minimax agent with alpha-beta pruning (question 3)
    """

    def getAction(self, gameState):
        """
          Returns the minimax action using self.depth and self.evaluationFunction
        """
        a = float('-inf')
        b = float('inf')
        v = float('-inf')
        utilities = []
        for action in gameState.getLegalActions(0):
            newState = gameState.generateSuccessor(0, action)
            currValue = self.minValue(newState, a, b, 1, 1) # 10
            utilities.append(currValue)
            v = max(v, currValue)
            a = max(a, v)

        maxVal = utilities[0]
        actionIndex = 0

        for i in range(len(utilities)):
            if utilities[i] > maxVal:
                maxVal = utilities[i]
                actionIndex = i

        return gameState.getLegalActions(0)[actionIndex]

    def maxValue(self, gameState, a, b, agentIndex, currDepth):
        currDepth += 1
        newAgentIndex = (agentIndex + 1) % gameState.getNumAgents()

        if gameState.isWin() or gameState.isLose():
            return self.evaluationFunction(gameState)
        if currDepth > self.depth:
            return self.evaluationFunction(gameState)

        v = float('-inf')

        for action in gameState.getLegalActions(agentIndex):
            successor = gameState.generateSuccessor(agentIndex, action)
            v = max(v, self.minValue(successor, a, b, newAgentIndex, currDepth))
            if v > b:
                return v
            a = max(a, v)

        return v

    def minValue(self, gameState, a, b, agentIndex, currDepth):
        newAgentIndex = (agentIndex + 1) % gameState.getNumAgents()

        if gameState.isWin() or gameState.isLose():
            return self.evaluationFunction(gameState)
        if currDepth > self.depth:
            return self.evaluationFunction(gameState)

        v = float('inf')

        for action in gameState.getLegalActions(agentIndex):
            successor = gameState.generateSuccessor(agentIndex, action)
            if newAgentIndex == 0:
                v = min(v, self.maxValue(successor, a, b, newAgentIndex, currDepth))
            else:
                v = min(v, self.minValue(successor, a, b, newAgentIndex, currDepth))
            if v < a:
                return v
            b = min(b, v)

        return v

class ExpectimaxAgent(MultiAgentSearchAgent):
    """
      Your expectimax agent (question 4)
    """

    def getAction(self, gameState):
        """
          Returns the expectimax action using self.depth and self.evaluationFunction

          All ghosts should be modeled as choosing uniformly at random from their
          legal moves.
        """
        utilities = []
        for action in gameState.getLegalActions(0):
            newState = gameState.generateSuccessor(0, action)
            utilities.append(self.expectimax(newState, 1, 1))

        max = utilities[0]
        actionIndex = 0

        for i in range(len(utilities)):
            if utilities[i] > max:
                max = utilities[i]
                actionIndex = i

        return gameState.getLegalActions(0)[actionIndex]

    def expectimax(self, gameState, agentIndex, currDepth):
        newAgentIndex = (agentIndex + 1) % gameState.getNumAgents()

        if agentIndex == 0:
            currDepth += 1

        if gameState.isWin() or gameState.isLose():
            return self.evaluationFunction(gameState)
        if currDepth > self.depth:
            return self.evaluationFunction(gameState)

        values = []
        for action in gameState.getLegalActions(agentIndex):
            successor = gameState.generateSuccessor(agentIndex, action)
            values.append(self.expectimax(successor, newAgentIndex, currDepth))

        if agentIndex == 0:
            return max(values)
        elif agentIndex > 0:
            average = sum(values) / len(values)
            return average


def betterEvaluationFunction(currentGameState):
    """
      Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
      evaluation function (question 5).

      DESCRIPTION: We calculated the distance to the nearest ghost, the nearest food,
      and the number of food, as well as the number of power pellets. We said it was generally
      bad to be near a ghost, good to be near food, extremely good to have less food, and
      extremely good to have fewer power pellets (so that pacman would want to eat the power
      pellets and then eat the ghosts)
    """

    position = currentGameState.getPacmanPosition()
    food = currentGameState.getFood()

    width = food.packBits()[0]
    height = food.packBits()[1]

    numGhosts = 0
    ghostDist = width + height

    for ghostPos in currentGameState.getGhostPositions():
        distance = util.manhattanDistance(ghostPos, position)
        if distance < ghostDist:
            ghostDist = distance
        numGhosts += 1

    foodDist = closestFood(position, food, width, height)
    foodTotal = totalFood(food, width, height)

    # When there are any ghosts who have scared time left, pursue them
    # Until there's only one scared amount left, then go back to running away
    numCaps = len(currentGameState.getCapsules())

    if ghostDist > 3:
        solution = 3 * (width + height) + ghostDist - 2 * foodDist - (width + height) * foodTotal- 100 * numCaps
    else:
        solution = (width + height) * ghostDist - foodDist - (width + height) * foodTotal- 100 * numCaps

    return solution

# Abbreviation
better = betterEvaluationFunction
