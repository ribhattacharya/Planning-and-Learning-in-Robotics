from statistics import mean
import numpy as np
import math
from numpy import average, loadtxt
import matplotlib.pyplot as plt
plt.ion()
import time
import os

from robotplanner import robotplanner
from targetplanner import targetplanner

# change parameters here
def params():
    """
    DESC: parameters used for modifying A* algorithm
    """
    skip = 50              # no. of points to use from the computed path (>=1)
    epsilon = 2           # epsilon value for the weigted A* algorithm
    return skip, epsilon


# functions to time how long planning takes  
def tic():
  return time.time()
def toc(tstart, nm=""):
  print('%s took: %s sec.\n' % (nm,(time.time() - tstart)))

def runtest(mapfile, robotstart, targetstart):
  # current positions of the target and robot
  robotpos = np.copy(robotstart);
  targetpos = np.copy(targetstart);
  euclidean_dist = []
  planning_time = []
  
  # environment
  envmap = loadtxt(mapfile)
    
  # draw the environment
  # transpose because imshow places the first dimension on the y-axis
  f, ax = plt.subplots()
  ax.imshow( envmap.T, interpolation="none", cmap='gray_r', origin='lower', \
             extent=(-0.5, envmap.shape[0]-0.5, -0.5, envmap.shape[1]-0.5) )
  ax.axis([-0.5, envmap.shape[0]-0.5, -0.5, envmap.shape[1]-0.5])
  ax.set_xlabel('x')
  ax.set_ylabel('y')  
  hr = ax.plot(robotpos[0], robotpos[1], 'bs')
  ht = ax.plot(targetpos[0], targetpos[1], 'rs')
  f.canvas.flush_events()
  plt.show()
  #uncomment following line if you want to create gif
  #plt.savefig('/Users/rishabhbhattacharya/Desktop/ECE276B_PR2/starter_code/results/gifs/gif3/%d.png' %(0))
  
  # now comes the main loop
  numofmoves = 0
  caught = False

  begin_time = tic()    # begin time for target capture
  for i in range(20000):
    # call robot planner
    t0 = tic()
    
    # custom logic for running aStar every skip steps
    skip, epsilon = params()
    if np.linalg.norm(robotpos - targetpos) > skip * 2:   #if target and robot are far away (2*skip euclidean distance)
        if i % skip == 0:                                 # compute A* every skipth move
            newrobotpos_list = robotplanner(envmap, robotpos, targetpos, skip, epsilon)

        newrobotpos = newrobotpos_list[i % skip]          # use new robot pos from the list
    else:                                                 # if robot is near the target, plan every move
        newrobotpos_list = robotplanner(envmap, robotpos, targetpos, skip, epsilon)
        newrobotpos = newrobotpos_list[0]

    planning_time.append(tic()-t0)                        # record time required for robotplanner to run
    
    
    # compute move time for the target, if it is greater than 2 sec, the target will move multiple steps
    movetime = max(1, math.ceil((tic()-t0)/2.0))
    
    #check that the new commanded position is valid
    if ( newrobotpos[0] < 0 or newrobotpos[0] >= envmap.shape[0] or \
         newrobotpos[1] < 0 or newrobotpos[1] >= envmap.shape[1] ):
      print('ERROR: out-of-map robot position commanded\n')
      break
    elif ( envmap[newrobotpos[0], newrobotpos[1]] != 0 ):
      print('ERROR: invalid robot position commanded\n')
      break
    elif (abs(newrobotpos[0]-robotpos[0]) > 1 or abs(newrobotpos[1]-robotpos[1]) > 1):
      print('ERROR: invalid robot move commanded\n')
      break

    # call target planner to see how the target moves within the robot planning time
    newtargetpos = targetplanner(envmap, robotpos, targetpos, targetstart, movetime)
    
    # make the moves
    robotpos = newrobotpos
    targetpos = newtargetpos
    euclidean_dist.append(np.linalg.norm(robotpos - targetpos))
    print('robot, target for move %d = (%d,%d), (%d,%d). Euclidean distance = %.2f' %(numofmoves,robotpos[0],robotpos[1],targetpos[0],targetpos[1], euclidean_dist[i]))
    numofmoves += 1

    hr.append(ax.plot(robotpos[0], robotpos[1], 'bs')) #store new robot positions to draw path later

    
    # draw positions
    # hr[0].set_xdata(robotpos[0])
    # hr[0].set_ydata(robotpos[1])
    # ht[0].set_xdata(targetpos[0])
    # ht[0].set_ydata(targetpos[1])
    # f.canvas.flush_events()
    # plt.show()
    #uncomment following line if you want to create gif
    #plt.savefig('/Users/rishabhbhattacharya/Desktop/ECE276B_PR2/starter_code/results/gifs/gif3/%d.png' %(i+1))
    
    # check if target is caught
    if (abs(robotpos[0]-targetpos[0]) <= 1 and abs(robotpos[1]-targetpos[1]) <= 1):
      print('robotpos = (%d,%d)' %(robotpos[0],robotpos[1]))
      print('targetpos = (%d,%d)' %(targetpos[0],targetpos[1]))
      caught = True
      toc(begin_time, 'With (skip, epsilon) = (%d,%d), target capture' %(skip, epsilon))
      break
  
  ht = ax.plot(targetpos[0], targetpos[1], 'rs')    #show final target position
  plt.show()
  # Plot data plots for comparison analysis
  plt.figure(2)
  plt.plot(range(numofmoves), euclidean_dist)
  plt.xlabel('No. of moves to target')
  plt.ylabel('Euclidean distance (map units)')
  plt.title('Target caught in %d moves (epsilon = %d)' %(numofmoves, epsilon))

  plt.figure(3)
  plt.plot(planning_time)
  plt.xlabel('No. of moves to target')
  plt.ylabel('Planning time per move (s)')
  plt.title('Avg. planning time = %.2e s (epsilon = %d)' %(mean(planning_time), epsilon))

  #np.savez('/Users/rishabhbhattacharya/Desktop/ECE276B_PR2/starter_code/results/epsilon_1/3c.npz', dist = euclidean_dist, time = planning_time)

  return caught, numofmoves


def test_map0():
  robotstart = np.array([0, 2])
  targetstart = np.array([5, 3])
  folder_path = os.path.dirname(os.path.abspath(__file__)) 
  filename = os.path.join(folder_path, 'maps/map0.txt')
  return runtest(filename, robotstart, targetstart)

def test_map1():
  robotstart = np.array([699, 799])
  targetstart = np.array([699, 1699])
  folder_path = os.path.dirname(os.path.abspath(__file__)) 
  filename = os.path.join(folder_path, 'maps/map1.txt')
  return runtest(filename, robotstart, targetstart)

def test_map2():
  robotstart = np.array([0, 2])
  targetstart = np.array([7, 9])
  folder_path = os.path.dirname(os.path.abspath(__file__)) 
  filename = os.path.join(folder_path, 'maps/map2.txt')
  return runtest(filename, robotstart, targetstart)
  
def test_map3():
  robotstart = np.array([249, 249])
  targetstart = np.array([399, 399])
  folder_path = os.path.dirname(os.path.abspath(__file__)) 
  filename = os.path.join(folder_path, 'maps/map3.txt')
  return runtest(filename, robotstart, targetstart)

def test_map4():
  robotstart = np.array([0, 0])
  targetstart = np.array([5, 6])
  folder_path = os.path.dirname(os.path.abspath(__file__)) 
  filename = os.path.join(folder_path, 'maps/map4.txt')
  return runtest(filename, robotstart, targetstart)

def test_map5():
  robotstart = np.array([0, 0])
  targetstart = np.array([29, 59])
  folder_path = os.path.dirname(os.path.abspath(__file__)) 
  filename = os.path.join(folder_path, 'maps/map5.txt')
  return runtest(filename, robotstart, targetstart)

def test_map6():
  robotstart = np.array([0, 0])
  targetstart = np.array([29, 36])
  folder_path = os.path.dirname(os.path.abspath(__file__)) 
  filename = os.path.join(folder_path, 'maps/map6.txt')
  return runtest(filename, robotstart, targetstart)

def test_map7():
  robotstart = np.array([1, 1])
  targetstart = np.array([4998, 4998])
  folder_path = os.path.dirname(os.path.abspath(__file__)) 
  filename = os.path.join(folder_path, 'maps/map7.txt')
  return runtest(filename, robotstart, targetstart)


def test_map1b():
  robotstart = np.array([249, 1199])
  targetstart = np.array([1649, 1899])
  folder_path = os.path.dirname(os.path.abspath(__file__)) 
  filename = os.path.join(folder_path, 'maps/map1.txt')
  return runtest(filename, robotstart, targetstart)

def test_map3b():
  robotstart = np.array([74, 249])
  targetstart = np.array([399, 399])
  folder_path = os.path.dirname(os.path.abspath(__file__)) 
  filename = os.path.join(folder_path, 'maps/map3.txt')
  return runtest(filename, robotstart, targetstart)

def test_map3c():
  robotstart = np.array([4, 399])
  targetstart = np.array([399, 399])
  folder_path = os.path.dirname(os.path.abspath(__file__)) 
  filename = os.path.join(folder_path, 'maps/map3.txt')
  return runtest(filename, robotstart, targetstart)
  

if __name__ == "__main__":

  # you should change the following line to test different maps
  caught, numofmoves = test_map1b()
  print('Number of moves made: {}; Target caught: {}.\n'.format(numofmoves, caught))
  plt.ioff()
  plt.show()