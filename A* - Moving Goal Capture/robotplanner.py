import numpy as np
import math
from pqdict import minpq

class environment():
    """
    DESC: Environment class that contains functions for parsing the environment
    """
    @staticmethod
    def getSuccessors(envmap, parentpos, targetpos):
        """
        DESC: returns successors node (except obstacles) and stage cost
        INPUT: environment, node, target
        OUTPUT: successor nodes, their stage cost
        """
        # all possible directions of the robot
        numofdirs = 8
        dX = [1, 1, 0, -1, -1, -1, 0, 1]
        dY = [0, 1, 1, 1, 0, -1, -1, -1]

        successor_node = []
        c = []

        for i in range(numofdirs):

            newx = parentpos[0] + dX[i]
            newy = parentpos[1] + dY[i]
            
            # is this node within map bounds?
            withinMapBound = (newx >= 0 and newx < envmap.shape[0] and newy >= 0 and newy < envmap.shape[1])
            
            if withinMapBound :
                if envmap[newx,newy] == 0:                      # is this node free (not an obstacle
                    successor_node.append((newx, newy))   # Store successor nodes tuple

                    # Create node for each successor if seen for the first time
                    if (newx, newy) not in X.node_dict:
                        X.node_dict[(newx, newy)] = Node(    X.num_nodes, 
                                                        np.array([newx, newy]), 
                                                        1E10, 
                                                        environment.getHeuristic([newx, newy], targetpos), 
                                                        np.array(parentpos), 
                                                        0
                                                    )
                
                        X.num_nodes += 1                # increase num_nodes after creation of any node

                    # save stage costs
                    if  i % 2 == 1:               # check diagonal moves
                        c.append(math.sqrt(2))
                    elif i % 2 == 0:                # check left, right, up, down moves
                        c.append(1)
            
        return successor_node, c
    
    
    @staticmethod
    def getHeuristic(nodepos, targetpos):
        """
        DESC: returns heuristic value 'h' for every node (Euclidean distance)
        INPUT: node, goal position
        OUTPUT: heuristic value
        """
        return np.linalg.norm(nodepos - targetpos)

class Node():
    def __init__(self, idx, nodepos, g, h, parentpos, CLOSED):
        """
        DESC: node id, node coordinates, g value, h value, parent coordinates, closed (true/false)
        """
        self.idx = idx
        self.nodepos = nodepos
        self.g = g
        self.h = h
        self.parentpos = parentpos
        self.CLOSED = CLOSED


class stateSpace():
    def __init__(self, robotpos, targetpos):
        """
        DESC: Initializes map state space (OPEN, node dictionary, num of nodes) and START, GOAL nodes
        INPUT: start, goal position
        """
        self.OPEN = minpq()
        self.node_dict = {}
        self.num_nodes = 1

        # Create START node
        self.node_dict[tuple(robotpos)] = Node(  self.num_nodes, 
                                            np.array(robotpos), 
                                            0, 
                                            environment.getHeuristic(robotpos, targetpos), 
                                            [None, None],           # no parent
                                            0
                                        )
        self.num_nodes += 1                                         # increase num_nodes after creation of any node

        # Create GOAL node
        self.node_dict[tuple(targetpos)] = Node(  self.num_nodes, 
                                            np.array(targetpos), 
                                            1E10, 
                                            0, 
                                            [None, None],           # no parent 
                                            0
                                        )
        self.num_nodes += 1                                         # increase num_nodes after creation of any node

        self.OPEN[tuple(robotpos)] = 0                              # put start node in OPEN, set 0 'f' value
    
class aStar():
    """
    DESC: class containing functions for the A* path finding algorithm
    """
    @staticmethod
    def solver(envmap, targetpos, epsilon):
        """
        DESC: solves for the optimal path using A* algorithm over the state space, changes node characteristics and sets the parent coordinates
        INPUT: environment, goal node
        """
        # run loop while target is not closed
        while not(X.node_dict[tuple(targetpos)].CLOSED):
            to_expand = X.OPEN.pop()                                # get priority node from OPEN, to_expand is a tuple
            X.node_dict[to_expand].CLOSED = 1                       # put this node in CLOSED

            # get all successors and their costs
            successors, cost = environment.getSuccessors(envmap, [to_expand[0], to_expand[1]], targetpos)

            for j in range(len(successors)):                        # loop over all ssuccessors to expand parent node

                child = successors[j]                               # elements of successor are tuple

                # if child is not closed and a shorter path is available
                if not(X.node_dict[child].CLOSED) and X.node_dict[child].g > X.node_dict[to_expand].g + cost[j]:
                        
                    # update g value and parent position for children
                    X.node_dict[child].g = X.node_dict[to_expand].g + cost[j]
                    X.node_dict[child].parentpos = X.node_dict[to_expand].nodepos

                    # Calculate f and update/ add child value to OPEN
                    X.OPEN[child] = X.node_dict[child].g + epsilon * X.node_dict[child].h



    @staticmethod
    def recoverPath(targetpos):
        """
        DESC: returns optimal path from START -> GOAL
        INPUT: goal position
        OUTPUT: heuristic value
        """
        node = X.node_dict[tuple(targetpos)].nodepos          # traverse backwards from target node
        path = np.array([]).reshape(0,2)                    # initialize empty path from GOAL -> START
        while not(np.all(node == [None, None])):
            path = np.append(path, node.reshape(1,2), axis=0).astype(int)   # append node to path
            node = X.node_dict[tuple(node)].parentpos                         # go to parent node

        path = np.flip(path, axis = 0)                      # flip path to obtain START -> GOAL path
        
        return path                                         # return path
        



def robotplanner(envmap, robotpos, targetpos, skip, epsilon):
    """
    DESC: Calls the required functions for computing the optimal path using A*
    INPUT: environment, robot position, goal position, skip value, epsilon value
    OUTPUT: User defined number of moves (depending on skip value)
    """

    global X
    X = stateSpace(robotpos, targetpos)                     # create State Space

    aStar.solver(envmap, targetpos, epsilon)                # Run A* solver
    nextmove = aStar.recoverPath(targetpos)                 # Recover path travelled

    return nextmove[1:skip+1]                               # return 'skip' moves in the optimal path                    