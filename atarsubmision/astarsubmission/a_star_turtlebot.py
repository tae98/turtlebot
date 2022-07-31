import ast
from sys import argv
from time import time
import cv2
import numpy as np
from src.world2d import Map
from src.constants import scaling_factor, angular_step
from math import sqrt
from queue import PriorityQueue
from src import constants

#parameter needed to be called to run the script
#startingNodeValue,goalPoinNodeValue= starting coordinate,goalCoordinate
#turtlebotData for clearance and wheel spin rate
script, startingNodeValue, goalPointNodeValue, turtlebotData, animation = argv
# node class for different types and characteristic of node
class Node:
    def __init__(self, nodeValue, ancestorNode, totalHeuristicCost, cost, neighbourPoints):
       
        #initializing nodes and properties used 
        self.nodeValue = nodeValue
        self.ancestor = ancestorNode
        self.neighbourNodes = neighbourPoints
        self.weight = totalHeuristicCost
        self.cost = cost
    
    #compares weight of different nodes
    def weighLesser(self, other):      
        val =self.weight < other.weight
        return val

    def __gt__(self, other):
        val =self.weight > other.weight
        return val
    
    def eqaul(self, other):
        val=self.weight == other.weight
        return val
    #get values of different node
    def getValNode(self):
        return self.nodeValue

    def getAncestorVal(self):
        return self.ancestor

    def getNeighbourNode(self):
        return self.neighbourNodes

    #calculating and assinging cost to nodes
    def getTotalCost(self):
        return self.weight

    def getCost(self):
        return self.cost

    def finalWeight(self, node_weight):
        self.weight = node_weight

    def fixCost(self, cost):
        self.cost = cost


def nodeSearch(mapObstacle, x, y):
    # validates where the coordinate lies
    #mapObstacle is 2d represnetaion of the world map drawn using python
    #x,y are the x,y coordinate values of the nodes being searched
    #the function will return false if the given coordinates and obstacles collide
    # if the coordinate is outside the boudnary of map dimention it returns false
    if x <= 0 or x >= constants.map_size[1] or y <= 0 or y >= constants.map_size[0]:
        return False
    # varifies to see if the coordinates are possible for the robot to be on
    elif mapObstacle[y, x].all() == 0:
        return False

    return True


def calcEuclidean(nodeA, nodeB):
     #eulidean distance calculation with nodeA being first coordinates and node B being second Coordinates
    val=sqrt((nodeB[0] - nodeA[0]) ** 2 + (nodeB[1] - nodeA[1]) ** 2)
    return val

def getCost(ancestorNode, cNode):
      # calculates cost for each children nodes using the coordinate value of ancestorial nodes
    val =ancestorNode.cost + calcEuclidean(ancestorNode.getValNode(), cNode)
    return val

class SearchAlgo:
    def __init__(self, initialNode, goalPointNode, wheelRotation, mapImage, animation):
        #initializing values needed to calculate the possible path between starting and goal coordinaates and 2d map created
        #wheelRotation is the rate per minute of 2 wheels of our turlebot
        # initializing the starting and goal points 
        self.initialNode = Node(initialNode, None, 0, 0, None)
        self.goalPointNode = goalPointNode
        # to create animation using cv2
        self.animation = animation
        # gets the wheels rotation rate 
        self.wheelRotation = wheelRotation
        # world map
        self.mapImage = mapImage[0]
        # scaled map with turtlebot size and clearance of obstacles
        self.mapObstacle = mapImage[1]
        # angular step size taken
        self.stepAng = constants.angular_step
        # size of the full map created
        self.map_size = constants.map_size[0], constants.map_size[1], (constants.total_angle // self.stepAng)
        #initialized list to store nodes and generated path
        self.nodeCurrentList = []
        self.posPath = []
        self.ancestor = np.full(fill_value=constants.no_parent, shape=self.map_size)
        #  format of the video created my cv2 and its saved location
        video_format = cv2.VideoWriter_fourcc(*'XVID')
        self.video_output = cv2.VideoWriter('path illustration.avi', video_format, 200.0,
                                            (self.map_size[1], self.map_size[0]))

    def calcHeuristic(self, node):
        #calculating using eculidean distance heuristic cost to goal
        cHeuristic= calcEuclidean(self.goalPointNode, node)
        return cHeuristic

    def calcWeight(self, node, cost):
        #final weight of the path adds up cost to the goal
        cWeight= self.calcHeuristic(node) + cost
        return cWeight

    def calcRobotOrientation(self, angle):
        #calcuates robot orientation (radian to maps orientation)
        # angle should be in range of 0-360
        pi2 = 2*np.pi
        if angle >= pi2:
            n = int(angle / (pi2))
            angle = angle - n * pi2
        elif angle < 0:
            # negative radian values to positive
            angle = abs(angle)
            if angle > pi2:
                n = int(angle / (pi2))
                angle = angle - n * pi2
        # change angle to degree
        angle = angle + (180 * angle / np.pi)
        if angle > constants.total_angle:
            angle = angle - constants.total_angle
        return int(angle / self.stepAng)

    def actionList(self, action):
         # list of possible movement that could be made
        if action == 0:
            val0 = 0, self.wheelRotation[0]
            return val0
        elif action == 1:
            val1 = 0, self.wheelRotation[1]
            return val1
        elif action == 2:
            val2= self.wheelRotation[0], 0
            return val2
        elif action == 3:
            val3= self.wheelRotation[1], 0
            return val3
        elif action == 4:
            val4 = self.wheelRotation[0], self.wheelRotation[0]
            return val4
        elif action == 5:
            val5= self.wheelRotation[1], self.wheelRotation[1]
            return val5
        elif action == 6:
            val6= self.wheelRotation[0], self.wheelRotation[1]
            return val6
        valx =self.wheelRotation[1], self.wheelRotation[0]
        return valx

    def move(self, ancestorNode, action):
        # getting the right movement for the robot to move from children nodes to parents node depending on its orientation
        pi2 = 2*np.pi
        wheelSpin = self.actionList(action)
        #wheel spin to wheels velocity for each side
        leftwheel = wheelSpin[0] * (pi2 / 60)
        rightwheel = wheelSpin[1] * (pi2 / 60)
        finalval=self.get_cNode(ancestorNode, leftwheel, rightwheel)
        return finalval
    
    def searchN(self):
        # Initializing different lists
        nodeList = PriorityQueue()
        initialNode = self.initialNode.getValNode()
        self.ancestor[int(initialNode[0])][int(initialNode[1])][int(initialNode[2])] = constants.start_parent
        # adding nodes
        nodeList.put(self.initialNode)
        # search loop
        while not nodeList.empty():
            # calculating nodes with minimum cost
            currentNode = nodeList.get()
            self.nodeCurrentList.append(currentNode)
            # add generated nodes to array and look for goal point
            if (self.calcHeuristic(currentNode.getValNode()) <= constants.goal_thresh or
                    currentNode.getValNode() == self.goalPointNode):
                self.posPath.append(currentNode)
                break
            for i in range(constants.max_actions):
                cNode = self.move(currentNode, i)
                if cNode is not None:
                    nodeList.put(cNode)

    def get_cNode(self, ancestorNode, leftwheelVel, rightwheelVel):

        #calucate the possilbe children nodes based on its motion of the wheel velocity
        # intialized emtpy array to store nodes
        initialP = []
        pathPossible = True
        #color used for graphic representation
        colorMediumGrey = [125, 125, 125]
        # coordinate and orientation of ancestor node
        ancestorNodeValue = ancestorNode.getValNode()
        y, x = ancestorNodeValue[0], ancestorNodeValue[1]
        angularPosition = np.pi * ancestorNodeValue[2] * self.stepAng / 180
        yPrevious, xPrevious, prev_angularPosition = y, x, ancestorNodeValue[2]
        # calculate lin and ang velocity in  m/s or rad/s
        xAxisLinearVelocity = 0.5 * constants.wheel_radius * (leftwheelVel + rightwheelVel) * np.cos(angularPosition) / constants.scaling_factor
        yAxisLinearVelocity = 0.5 * constants.wheel_radius * (leftwheelVel + rightwheelVel) * np.sin(angularPosition) / constants.scaling_factor
        linearightwheelVel = sqrt((xAxisLinearVelocity ** 2) + (yAxisLinearVelocity ** 2))
        angularightwheelVel = (constants.wheel_radius / constants.wheel_distance) * (rightwheelVel - leftwheelVel)
        t = 0
        while t < constants.total_time:
            t += constants.time_step
            # calculate new coordinate af the time pass
            x += (0.5 * constants.wheel_radius * (leftwheelVel + rightwheelVel) * np.cos(angularPosition) * constants.time_step *
                  constants.time_scaling)
            y += (0.5 * constants.wheel_radius * (leftwheelVel + rightwheelVel) * np.sin(angularPosition) * constants.time_step *
                  constants.time_scaling)
            angularPosition += ((constants.wheel_radius / constants.wheel_distance) * (rightwheelVel - leftwheelVel) *
                      constants.time_step * constants.time_scaling)
            #get current position information to look for ancestor nodes in the map
            angularPositionUpdate = self.calcRobotOrientation(angularPosition)
            #adding valid intermidiat nodes to the array
            if (nodeSearch(self.mapObstacle, int(x), self.map_size[0] - int(y)) and
                    self.ancestor[int(y)][int(x)][angularPositionUpdate] == constants.no_parent):
                initialP.append(([yPrevious, xPrevious, prev_angularPosition], [y, x, angularPosition], [linearightwheelVel, angularightwheelVel]))
                xPrevious, yPrevious, prev_angularPosition = x, y, angularPosition
            else:
                pathPossible = False
                break
        # if intermidiate path is in colision with obstacle start over
        if pathPossible:
            nodeL = None
            # base cost for the children node
            cost = ancestorNode.getCost()
            lenghtInitialP = len(initialP)
            # Add exploration to video
            for i in range(lenghtInitialP):
                previousVisitedNode, currentNode, _ = initialP[i]
                cost += calcEuclidean(previousVisitedNode, currentNode)
                # index of the intermidiate nodes and update its parent
                previousVisitedNode[2] = self.calcRobotOrientation(previousVisitedNode[2])
                currentNode[2] = self.calcRobotOrientation(currentNode[2])
                self.ancestor[int(currentNode[0])][int(currentNode[1])][int(currentNode[2])] = \
                    np.ravel_multi_index([int(previousVisitedNode[0]), int(previousVisitedNode[1]), int(previousVisitedNode[2])], dims=self.map_size)
                # visual representation of the created node
                if self.animation:
                    cv2.arrowedLine(self.mapImage, (int(previousVisitedNode[1]), self.map_size[0] - int(previousVisitedNode[0])),
                                    (int(currentNode[1]), self.map_size[0] - int(currentNode[0])), colorMediumGrey)
                    self.video_output.write(self.mapImage)
                # define last node in the calc to be the children node
                if i == lenghtInitialP - 1:
                    nodeL = Node(currentNode, ancestorNodeValue, float('inf'), float('inf'), initialP)
                    nodeL.fixCost(cost)
                    nodeL.finalWeight(self.calcWeight(currentNode, nodeL.cost))
            return nodeL
        return None

    def printPath(self):
        #back track and creates the path with minimal heuristic cost
        #if fail to search then terminate
        if not len(self.posPath):
            print('No path found')
            return False
        red = [0, 0, 255]
        blue = [255, 0, 0]
        green = [0, 255, 0]
        # Create texte file with linear and angular velocity
        pathTxtGenerated = open('generatedPath/astarPath.txt', 'w+')
        # back track from nearest nodes to the goal
        nodeL = self.posPath[0]
        print('Calculating for Path')
        #continue until it reaches initial position
        while nodeL.getValNode() != self.initialNode.getValNode():
            for node in self.nodeCurrentList:
                if node.getValNode() == nodeL.getAncestorVal():
                    self.posPath.append(node)
                    nodeL = node
                    break
        print('Path found')
        # continue for path node
        for i in range(len(self.posPath) - 2, -1, -1):
            # get list of path nodes for the robot to travel
            current_sub_nodes = self.posPath[i].getNeighbourNode()
            for j in range(0, len(current_sub_nodes)):
                currentNodeValue = current_sub_nodes[j]
                pathTxtGenerated.write(str(currentNodeValue[2][0]) + ',' + str(currentNodeValue[2][1]) + '\n')
                if self.animation:a
                    cv2.line(self.mapImage,
                             (int(currentNodeValue[0][1]), self.map_size[0] - int(currentNodeValue[0][0])),
                             (int(currentNodeValue[1][1]), self.map_size[0] - int(currentNodeValue[1][0])),
                             blue)
                    self.video_output.write(self.mapImage)
        if self.animation:
            # video reperesentation red circle for goal and green for start
            cv2.circle(self.mapImage,
                       (int(self.posPath[-1].getValNode()[1]), self.map_size[0] - int(self.posPath[-1].getValNode()[0])),
                       int(constants.robot_radius), red, -1)
            cv2.circle(self.mapImage,
                       (int(self.posPath[0].getValNode()[1]), self.map_size[0] - int(self.posPath[0].getValNode()[0])),
                       int(constants.robot_radius), green, -1)
            # duration for the path
            for _ in range(1000):
                self.video_output.write(self.mapImage)
        return True



if __name__ == '__main__':
    # Convert input arguments into tuples
    turtlebotData = tuple(ast.literal_eval(turtlebotData))
    startingNodeValue = tuple(ast.literal_eval(startingNodeValue))
    goalPointNodeValue = tuple(ast.literal_eval(goalPointNodeValue))
    # Initialize the map class and get map image to check for obstacles
    obstacle_map = Map( turtlebotData[2])
    check_image = obstacle_map.mapObstacle
    # Convert start and goal nodes given by user into coordinates from map frame
    startingNodeValue = obstacle_map.getMapPosition((startingNodeValue[0], startingNodeValue[1]), startingNodeValue[2])
    goalPointNodeValue = obstacle_map.getMapPosition(goalPointNodeValue)
    # Check validity of start and goal nodes
    if not (nodeSearch(check_image, startingNodeValue[1], obstacle_map.height - startingNodeValue[0])
            and nodeSearch(check_image, goalPointNodeValue[1], obstacle_map.height - goalPointNodeValue[0])):
        print('Fail to find appropriate path')
        quit()
    # Initialize the astarsearch class to find the goal node
    # Initialize astarsearch only after checking start and goal points
    astarsearch = SearchAlgo(startingNodeValue, goalPointNodeValue, (turtlebotData[0], turtlebotData[1]),
                        (obstacle_map.mapImage, check_image), int(animation))
    # Start exploration
    astarsearch.searchN()
    start_time = time()
    astarsearch.printPath()
    print('Search Finished:', time() - start_time)
