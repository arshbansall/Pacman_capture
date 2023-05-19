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


# imports...
from captureAgents import CaptureAgent
import random, time, util
from game import Directions
import game


import random, util
from captureAgents import CaptureAgent


#################
# Team creation #
#################


def createTeam(firstIndex, secondIndex, isRed,
               first='offenseAgent', second='defenseAgent'):
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



class search(CaptureAgent):

    # from baselineTeam.py
    def getFeatures(self, gameState):
        """
        Returns a counter of features for the state
        """
        features = util.Counter()
        features['successorScore'] = self.getScore(gameState)
        return features

    # from baselineTeam.py
    def getWeights(self, gameState):
        """
        Normally, weights do not depend on the gamestate.  They can be either
        a counter or a dictionary.
        """
        return {'successorScore': 1.0}

    # from baselineTeam.py
    def evaluate(self, gameState):
        """
        Computes a linear combination of features and feature weights
        """
        features = self.getFeatures(gameState)
        weights = self.getWeights(gameState)
        return features * weights

    #some components taken and modeled from baselineTeam.py
    def registerInitialState(self, gameState):
        self.start = gameState.getAgentPosition(self.index)
        CaptureAgent.registerInitialState(self, gameState)

        self.myFood = None #food on my side
        self.foodTimer = 0

        #find mid point of my side
        self.center = (int(gameState.data.layout.width / 2), int(gameState.data.layout.height / 2))

        i = 0
        myTeam = [] #list of agents on team
        while len(myTeam) < gameState.getNumAgents():
            myTeam.append(i) #add to my team
            i += 1
            myTeam.sort()
        self.registerTeam(myTeam) #register team


    #from baselineTeam.py
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


    def alphaBeta(self, gameState, agent, depth, opponents, a, b):
        """
        Alpha b algorithm to calculate closest position to known agents and food
        """

        # if at last agent, restart
        if agent >= gameState.getNumAgents():
            agent = 0
        # If not a known, visible opponent:
        if agent not in opponents:
            return self.alphaBeta(gameState, agent + 1, depth, opponents, a, b) #move no next layer

        #if game is over, or at ddepth of 1 or the current agent is selected
        if gameState.isOver() or depth == 1  or agent == 0:
           return self.evaluate(gameState)
        else:
            self.alphaBeta(gameState, agent + 1, depth, opponents, a, b)

        nextAgent = agent + 1 #go to next agent
        actions = gameState.getLegalActions(agent) #get legal actions

        if agent == 0:
            depth += 1# add depth

        #if agent is on team
        if agent in self.agentsOnTeam: #if agent on team, find min
            v = -100000000000
            for action in actions:
                if v < self.alphaBeta(gameState.generateSuccessor(agent, action), nextAgent, depth, opponents, a, b): #is new v bigger?
                    v = self.alphaBeta(gameState.generateSuccessor(agent, action), nextAgent, depth, opponents, a, b)
                if v > b: #if new beta
                    return v
                a = min(a, v)
            return v
        else: #else, find max
            v = 1000000000000
            for action in actions:
                if v < self.alphaBeta(gameState.generateSuccessor(agent, action), nextAgent, depth, opponents, a, b):
                    v = self.alphaBeta(gameState.generateSuccessor(agent, action), nextAgent, depth, opponents, a, b)
                if v < a: #if new alpha
                    return v
                b = max(b, v) #find max
            return v


    def chooseAction(self, gameState):
        a = -100000000000000
        b = 100000000000000
        # Find opponents
        opponents = [a for a in range(gameState.getNumAgents()) if gameState.getAgentState(a).getPosition() != None]
        # tuple of weight and action
        v = (-100000000000, 'None')
        actions = gameState.getLegalActions(self.index)
        depth = 0

        for action in actions:
            if v[0] < (self.alphaBeta(gameState.generateSuccessor(self.index, action), 1 + self.index, depth, opponents, a, b)):
                v = (self.alphaBeta(gameState.generateSuccessor(self.index, action), 1 + self.index, depth, opponents, a, b), action)
            if v[0] > b:  # if new max
                return v[1]  # return action
            a = max(a, v[0])
        return v[1]



class defenseAgent(search):
    """
    Defensive agent that searches for visible pacmans, otherwise patrols around center of side by visiting foods.
    Components of the baselineTeam.py file were replecated here and optimized. Each section method indicates
    when this is true
    """
    #some parts of this method come from baselineTeam.py
    def getFeatures(self, gameState):
        features = util.Counter()
        foodList = self.getFoodYouAreDefending(gameState).asList()  # find food on my side
        myState = gameState.getAgentState(self.index)
        myPos = myState.getPosition()

        # Computes distance to invaders we can see
        enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)] #list of opponents
        invaders = [a for a in enemies if a.isPacman and a.getPosition() != None] #opponents on offense
        features['numInvaders'] = len(invaders) #number of offensive opponents
        foodDistance = 0

        # if there are any invaders:
        if len(invaders) > 0:
            dists = [self.getMazeDistance(myPos, a.getPosition()) for a in invaders]
            features['invaderDistance'] = max(dists) #go to invaders
        else: #if no invaders, patrol
            if self.myFood: #if there is food on my side
                foodDistance = self.getMazeDistance(myPos, self.myFood) #stay around that food

            #find new food on my side
            if foodDistance == 0 or self.foodTimer == 0 : #if time is up or no known invaders
                random.shuffle(foodList) #shuffle food list to get random close food to patrol around
                for food in foodList: #for known food
                    if self.getMazeDistance(self.center, food) < 13: #if food is close
                        self.myFood = food #set as my food
                    self.foodTimer -= 1  # count down timer for defense agent

            foodDistance = self.getMazeDistance(myPos, self.myFood)#find distance to that food
            features['invaderDistance'] = foodDistance #go to that food
            self.foodTimer = 10  # reset counter to search for new food
        return features

    def getWeights(self, gameState):
        return {'numInvaders': -1000, 'invaderDistance': -10, 'stop': -100, 'reverse': -2}



class offenseAgent(search):
    """
    offensive agent that searches for food while avoiding opponents that are defenders (ghosts).
    Components of the baselineTeam.py file were replecated here and optimized. Each section method indicates
    when this is true
    """

    #portions of this method are taken from baselineTeam.py
    def getFeatures(self, gameState):
        features = util.Counter()
        foodList = self.getFood(gameState).asList() #list of food collected
        features['distanceToGhost'] = 0
        features['successorScore'] -= len(foodList) #score is num of food
        #find opponents
        opponents = [a for a in range(gameState.getNumAgents()) if gameState.getAgentState(a).getPosition() != None]
        #get visibile opponents
        visOpponents = self.getOpponents((gameState)) #opponents that are visible to pacman
        isPacman = gameState.getAgentState(self.index).isPacman #true if the agent is on offense
        if gameState.getAgentState(self.index).scaredTimer > 0: #if powerup collected
            isScared = True
        else:
            isScared = False

        if gameState.getAgentState(self.index).numCarrying < 3: #if pacman is carrying less than 3 food, search for more
            if len(foodList) > 0: #if have food
                myPos = gameState.getAgentState(self.index).getPosition() #get position
                minDistance = min([self.getMazeDistance(myPos, food) for food in foodList]) #find closest food
                features['distanceToFood'] = minDistance
            else: #else go back to start
                myPos = gameState.getAgentState(self.index).getPosition()
                features['distanceToFood'] = self.getMazeDistance(myPos, self.start)
        else: #otherwise, deposit on my side
            myPos = gameState.getAgentState(self.index).getPosition()
            features['distanceToFood'] = self.getMazeDistance(myPos, self.start)

        if set(visOpponents) in opponents: # Check if any visible opponent agents.
            if isPacman: #if agent is on offense
                if isScared: #if power up collected and ghosts scared
                    for opponent in visOpponents:
                        opponentPos = gameState.getAgentState(opponent).getPosition() #find opponent
                        opponentDistance = self.getMazeDistance(myPos, opponentPos)
                        features['distanceToOpponent'] = opponentDistance
                else: #get away
                    centerDistance = self.getMazeDistance(self.center, myPos) #go back to center
                    features['distanceToOpponent'] = centerDistance
        return features

    def getWeights(self, gameState):
        return {'successorScore': 100, 'distanceToFood': -1, 'distanceToOpponent': 1}





