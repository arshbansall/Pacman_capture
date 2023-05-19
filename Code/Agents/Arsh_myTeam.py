# myTeam.py
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


from captureAgents import CaptureAgent
import random, time, util
from game import Directions
import game
from util import nearestPoint
from game import Actions

#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first = 'OffensiveReflexAgent', second = 'DefensiveReflexAgent'):
  """
  This function should return a list of two agents that will form the
  team, initialized using firstIndex and secondIndex as their agent
  index numbers.  isRed is True if the red team is being created, and
  will be False if the blue team is being created.

  As a potentially helpful development aid, this function can take
  additional string-valued keyword arguments ("first" and "second" are
  such arguments in the case of this function), which will come from
  the --redOpts and --blueOpts command-line arguments to capture.py.
  For the nightly contest, however, your team will be created without
  any extra arguments, so you should make sure that the default
  behavior is what you want for the nightly contest.
  """

  # The following line is an example only; feel free to change it.
  return [eval(first)(firstIndex), eval(second)(secondIndex)]

##########
# Agents #
##########

class SearchAgent(CaptureAgent):
  """
  A Dummy agent to serve as an example of the necessary agent structure.
  You should look at baselineTeam.py for more details about how to
  create an agent as this is the bare minimum.
  """

  def registerInitialState(self, gameState):

    self.start = gameState.getAgentPosition(self.index)
    self.center = (int(gameState.data.layout.width / 2), int(gameState.data.layout.height / 2))

    CaptureAgent.registerInitialState(self, gameState)

  def getSuccessor(self, gameState, action):
    """
    Finds the next successor which is a grid position (location tuple).
    """
    successor = gameState.generateSuccessor(self.index, action)
    pos = successor.getAgentState(self.index).getPosition()
    if pos != nearestPoint(pos):
      # Only half a grid position was covered
      return successor.generateSuccessor(self.index, action)
    else:
      return successor

  def getCostOfActions(self, gameState, actions):
    """Returns the cost of a particular sequence of actions.  If those actions
    include an illegal move, return 999999"""
    x,y= self.start
    cost = 0
    for action in actions:
        dx, dy = Actions.directionToVector(action)
        x, y = int(x + dx), int(y + dy)
        if gameState.hasWall(x, y):
            return 999999
        cost += 1
    return cost

  def isGoalState(self, gameState, pos):
    food = self.getFood(gameState)
    return food[int(pos[0])][int(pos[1])]

  def nullHeuristic(state, x, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

  # A* Algorithm 
  def aStarSearch(self, gameState, heuristic=nullHeuristic, goalState=isGoalState, bestIntialAction=[]):
    start_state = self.start 
    start_state_astar_score = self.getCostOfActions(gameState, bestIntialAction) + heuristic(gameState, start_state) # Gets cost of actions and heuristics value for the beginning gameState.
    if goalState(gameState, start_state): # Checks whether beginning gameState is GoalState.
        return []
    frontier = util.PriorityQueue() # Creating a Priority Queue.
    frontier.push((gameState, bestIntialAction, start_state_astar_score), start_state_astar_score) 
    explored = dict()

    while not frontier.isEmpty(): # Iterates over the queue until a goal state with minimum cost is found.
      state, actions, cost = frontier.pop()
      pos = state.getAgentState(self.index).getPosition()

      if pos in explored and explored[pos] <= cost:
        continue
      if goalState(gameState, pos): 
        return actions

      explored[pos] = cost

      for action in state.getLegalActions(self.index): # Get action for next state.
        successor =  self.getSuccessor(state, action)
        successorPos = successor.getAgentState(self.index).getPosition()
        child_astar_score = self.getCostOfActions(successor, actions + [action]) + heuristic(successor, successorPos)
        child = (successor, actions + [action], child_astar_score)
        frontier.update(child, child_astar_score)
  
  def chooseAction(self, gameState):
    """
    Picks among actions randomly.
    """
    actionsTemp = gameState.getLegalActions(self.index)
    return random.choice(actionsTemp)
    '''
    You should change this in your own agent.
    '''

#CODE for Offensive Agent
class OffensiveReflexAgent(SearchAgent):

  def registerInitialState(self, gameState): #Initialization of basic variables

    self.start = gameState.getAgentPosition(self.index)
    self.center = (int(gameState.data.layout.width / 2), int(gameState.data.layout.height / 2))

    self.actionsOffence = [] 
    self.counterOffence = 0
    self.actionsExhaustedOffence = True

    self.actionsHome = []
    self.counterHome = 0
    self.actionsExhaustedHome = True

    self.chaseFood = True
    self.goBackHome = False

    CaptureAgent.registerInitialState(self, gameState)

  def evaluate(self, gameState, action):
    """
    Computes a linear combination of features and feature weights
    """
    features = self.getFeatures(gameState, action)
    weights = self.getWeights(gameState, action)

    return features * weights
  
  def getFeatures(self, gameState, action):
    features = util.Counter()
    successor = self.getSuccessor(gameState, action)
    foodList = self.getFood(successor).asList()    
    features['successorScore'] = -len(foodList)
    currentPosition = gameState.getAgentState(self.index).getPosition()

    opponents = [a for a in range(gameState.getNumAgents()) if gameState.getAgentState(a).getPosition() != None]
    
    visOpponents = self.getOpponents((gameState)) # opponents visible to pacman

    if set(visOpponents) in opponents and gameState.getAgentState(self.index).isPacman: # Checks whether there is a ghost nearby during offense action
      self.chaseFood = False # Changes goal from Chasing Food -> Go Back Home
      self.goBackHome = True # Changes goal from Chasing Food -> Go Back Home
      centerDistance = self.getMazeDistance(self.center, currentPosition) 
      features['distanceToOpponent'] = centerDistance

    elif gameState.getAgentState(self.index).numCarrying >= 1: # Checks whether the offense agent has eaten one food
      self.chaseFood = False # Changes goal from Chasing Food -> Go Back Home
      self.goBackHome = True # Changes goal from Chasing Food -> Go Back Home
      centerDistance = self.getMazeDistance(self.center, currentPosition)
      features['distanceToOpponent'] = centerDistance

    else:
      self.chaseFood = True # Changes goal from Go Back Home -> Chasing Food
      self.goBackHome = False # Changes goal from Go Back Home -> Chasing Food
      myPos = gameState.getAgentState(self.index).getPosition() 
      minDistance = min([self.getMazeDistance(myPos, food) for food in foodList]) 
      features['distanceToFood'] = minDistance       

    return features 

  # Goal State definition when chaseFood = True
  def offenseFoodGoalState(self, gameState, pos): 
    food = self.getFood(gameState) 
    return food[int(pos[0])][int(pos[1])]

  # Goal State definition when goBackHome = True
  def offenseHomeGoalState(self, gameState, pos): 
    if self.red:
      return pos == (self.center[0] - 2, self.center[1])
    else:
      return pos == (self.center[0] + 2, self.center[1])

  # Heuristic definition when chaseFood = True
  def offenseFoodHeuristic(self, gameState, pos1): 
    heuristics = []
    foodGrid = self.getFood(gameState)

    for food in foodGrid.asList():
      h = self.getMazeDistance(pos1, food)
      heuristics.append(h)

    if len(heuristics) > 0:
      return min(heuristics)
    else:
      return 0

  def chooseAction(self, gameState):

    # Reseting all variables when the offensive agent dies
    if gameState.getAgentState(self.index).getPosition() == self.start:
      self.actionsOffence = []
      self.counterOffence = 0
      self.actionsExhaustedOffence = True

      self.actionsHome = []
      self.counterHome = 0
      self.actionsExhaustedHome = True

    actions = gameState.getLegalActions(self.index)

    self.getFeatures(gameState, actions[0]) # Gets the current action whether to chase food or go Back home

    if self.chaseFood: # if Chase Food is true

      self.actionsExhaustedHome = True
      self.counterHome = 0

      if self.actionsExhaustedOffence: 

        self.actionsOffence = self.aStarSearch(gameState, heuristic=self.offenseFoodHeuristic, goalState=self.offenseFoodGoalState) # Create a sequence of actions for the nearest food
        self.actionsExhaustedOffence = False
      
      if self.actionsOffence != None and self.counterOffence < len(self.actionsOffence): # Send the sequence of actions one after another until all of the actions are sent
        action = self.actionsOffence[self.counterOffence] 
        self.counterOffence += 1
        return action
      else:  # Send a rnadom action and set the requirement for the Astar sequences to true 
        self.actionsExhaustedOffence = True
        self.counterOffence = 0 
        actionsTemp = gameState.getLegalActions(self.index)
        return random.choice(actionsTemp)

    elif self.goBackHome:

      self.actionsExhaustedOffence = True
      self.counterOffence = 0

      if self.actionsExhaustedHome:
        self.actionsHome = self.aStarSearch(gameState, heuristic=self.nullHeuristic, goalState=self.offenseHomeGoalState, bestIntialAction=[]) # Create a sequence of actions for the entry point in home territory.
        self.actionsExhaustedHome = False
      
      if self.actionsHome != None and self.counterHome < len(self.actionsHome): # Send the sequence of actions one after another until all of the actions are sent
        action = self.actionsHome[self.counterHome]
        self.counterHome += 1
        return action
      else: # Send a rnadom action and set the requirement for the Astar sequences to true 
        self.actionsExhaustedHome = True
        self.counterHome = 0 
        actionsTemp = gameState.getLegalActions(self.index)
        return random.choice(actionsTemp)

  def getWeights(self, gameState, action):
    return {'successorScore': 100, 'distanceToFood': -1, 'distanceToOpponent': 1}

#CODE for Defensive Agent
class DefensiveReflexAgent(SearchAgent):

  def registerInitialState(self, gameState):
    self.start = gameState.getAgentPosition(self.index)
    self.center = (int(gameState.data.layout.width / 2), int(gameState.data.layout.height / 2))

    self.actionsDefenseChase = []
    self.counterDefenseChase = 0
    self.actionsExhaustedDefense = True

    self.actionsPatrol = []
    self.counterPatrol = 0
    self.actionsExhaustedPatrol = True

    self.chaseEnemy = False
    self.Patrol = True

    CaptureAgent.registerInitialState(self, gameState)
  
  def evaluate(self, gameState, action):
    features = self.getFeatures(gameState, action)
    weights = self.getWeights(gameState, action)

    return features * weights

  def getFeatures(self, gameState, action):
    features = util.Counter()
    successor = self.getSuccessor(gameState, action)

    myState = successor.getAgentState(self.index)
    myPos = myState.getPosition()

    # Computes distance to invaders we can see
    enemies = [successor.getAgentState(i) for i in self.getOpponents(successor)]
    invaders = [a for a in enemies if a.isPacman and a.getPosition() != None]
    features['numInvaders'] = len(invaders)

    if len(invaders) <= 0: # If there are no invaders 
      self.Patrol = True # Changes goal from Chasing Enemy -> Patroling
      self.chaseEnemy = False # Changes goal from Chasing Enemy -> Patroling
      features['invaderDistance'] = 0
    elif len(invaders) > 0:  # If there are invaders
      self.Patrol = False # Changes goal from Patroling -> Chasing Enemy
      self.chaseEnemy = True # Changes goal from Patroling -> Chasing Enemy
      features['invaderDistance'] = 0

    return features

  # Heuristic calculation of the distance between the invader and defense agent
  def defenseHeuristic(self, gameState, pos):
    enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
    invaders = [a for a in enemies if a.isPacman and a.getPosition() != None]
    heuristics = []

    for invader in invaders:
      h = self.getMazeDistance(pos, invader.getPosition())
      heuristics.append(h)

    if len(heuristics) > 0:
      return min(heuristics)
    else:
      return 0
  
  # Goal State definition when Patoling = True
  def defenseHomeGoalState(self, gameState, pos):
    if self.red:
      return pos == (self.center[0] - 2, self.center[1]) # Sets center position of red map as goal state when agent is in the red team
    else:
      return pos == (self.center[0] + 2, self.center[1]) # Sets center position of blue map as goal state when agent is in the blue team

  # Goal State definition when Chasing Enemy = True
  def defenseGoalState(self, gameState, pos):
    enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
    invaders = [a for a in enemies if a.isPacman and a.getPosition() != None]
    for invader in invaders:
      if invader.getPosition() == pos: # Sets invader postion as goal state
        return True
    return False

  def chooseAction(self, gameState):

    actions = gameState.getLegalActions(self.index)
    self.getFeatures(gameState, actions[0]) # Gets the current action whether to chase enemy or keep patroling

    if self.Patrol: # If Patroling is the current requirement
      self.actionsExhaustedDefense = True
      self.counterDefenseChase = 0

      if self.actionsExhaustedPatrol:
        self.actionsPatrol = self.aStarSearch(gameState, heuristic=self.nullHeuristic, goalState=self.defenseHomeGoalState) # Creates a sequence of actions to reach the center
        self.actionsExhaustedPatrol = False
      
      if self.actionsPatrol != None and self.counterPatrol < len(self.actionsPatrol): # Send the sequence of actions one after another until all of the actions are sent
        action = self.actionsPatrol[self.counterPatrol]
        self.counterPatrol += 1
        return action
      else:   # Send a rnadom action and set the requirement for the Astar sequences to true 
        self.actionsExhaustedPatrol = True
        self.counterPatrol = 0 
        actionsTemp = gameState.getLegalActions(self.index)
        return random.choice(actionsTemp)

    elif self.chaseEnemy: # If chasing the enemy is the current requirement
      self.actionsExhaustedPatrol = True
      self.counterPatrol = 0

      if self.actionsExhaustedDefense:
        self.actionsDefenseChase = self.aStarSearch(gameState, heuristic=self.defenseHeuristic, goalState=self.defenseGoalState) # Creates a sequence of actions to reach and eat the invader
        self.actionsExhaustedDefense = False
      
      if self.actionsDefenseChase != None and self.counterDefenseChase < len(self.actionsDefenseChase): # Send the sequence of actions one after another until all of the actions are sent
        action = self.actionsDefenseChase[self.counterDefenseChase]
        self.counterDefenseChase += 1
        return action
      else: # Send a rnadom action and set the requirement for the Astar sequences to true 
        self.actionsExhaustedDefense = True
        self.counterDefenseChase = 0 
        actionsTemp = gameState.getLegalActions(self.index)
        return random.choice(actionsTemp)

  def getWeights(self, gameState, action):
    return {'numInvaders': -1000, 'onDefense': 100, 'invaderDistance': -10}