import numpy as np
import gym
from utils import *
from example import example_use_of_gym_env
from itertools import product
from tqdm import tqdm


# Control input mappings
MF = 0 # Move Forward
TL = 1 # Turn Left
TR = 2 # Turn Right
PK = 3 # Pickup Key
UD = 4 # Unlock Door

dir = {                 # agent orientation mapping
    0: np.array([1,0]),   # Right direction
    1: np.array([0,1]),   # Down direction  
    2: np.array([-1,0]),  # Left direction
    3: np.array([0,-1])   # Up direction
}

env_dict = {            # environment list
        1: './envs/doorkey-5x5-normal.env',
        2: './envs/doorkey-6x6-normal.env',
        3: './envs/doorkey-8x8-normal.env',
        
        4: './envs/doorkey-6x6-direct.env',
        5: './envs/doorkey-8x8-direct.env',
        
        6: './envs/doorkey-6x6-shortcut.env',
        7: './envs/doorkey-8x8-shortcut.env'
}

highCost = 1E5  #set high cost for invalid/undesirable actions

key_loc = {             # Key location dictionary
    0: np.array([1,1]),
    1: np.array([2,3]), 
    2: np.array([1,6]) 
}

key_loc_inv = {         # Inverse key location dictionary
    '11':0,
    '23':1, 
    '16':2 
}

goal_loc = {            # Goal location dictionary
    0: np.array([5,1]),
    1: np.array([6,3]),    
    2: np.array([5,6])  
}

goal_loc_inv = {        # Inverse key location dictionary
    '51':0,
    '63':1, 
    '56':2 
}

class problem_A():
    def initvalueFunc(self, states, goal):
        '''
        DESC: Initializes the value function for x_T: highCost everywhere and 0 at the goals
        INPUT: 1x5 vector containing all states, goal position
        OUTPUT: V_T (initial value function)
        '''
        V = np.zeros(states.shape[0]) + highCost # initialize V with very high cost
        idx = np.all(states[:,:2] == goal, axis = 1) # find states with goal position
        V[idx] = 0                  # goal positions have zero cost
        return V

    def stageCost(self, state, act, env):
        '''
        DESC: Computes cost for a given state and input
        INPUT: 1x5 vector containing all states, action, environemnt
        OUTPUT: cost
        '''

        # Initialize values
        agent_pos = np.array([state[0], state[1]])  # agent position in grid
        agent_dir = dir[state[2]]                   # agent pointer direction
        key_status = state[3]                       # key status
        door_status = state[4]                      # door status

        front_cell = agent_pos + agent_dir          #calculate front cell

        # Initialize values for conditional checks ahead
        try:
            wall_ahead = isinstance(env.grid.get(front_cell[0], front_cell[1]), gym_minigrid.minigrid.Wall)
        except AssertionError:
            wall_ahead = True                       # check if wall ahead, treat out of map as wall
        
        try:
            key_ahead = isinstance(env.grid.get(front_cell[0], front_cell[1]), gym_minigrid.minigrid.Key)
        except AssertionError:
            key_ahead = False                       # check if key ahead
        
        try:
            door_ahead = isinstance(env.grid.get(front_cell[0], front_cell[1]), gym_minigrid.minigrid.Door)
        except AssertionError:
            door_ahead = False                      # check if door ahead

        try:
            at_goal = isinstance(env.grid.get(agent_pos[0], agent_pos[1]), gym_minigrid.minigrid.Goal)
        except AssertionError:
            at_goal = False                         # check if at goal

        
        # Conditions for stage cost

        # don't do anything if at goal
        if at_goal:
            cost = 0
            
        # avoid going into walls   
        elif wall_ahead and act == MF:
            cost = highCost 
        
        # picked key without facing it, picked key while having key
        elif (not(key_ahead) or key_status == 1) and act == PK:
            cost = highCost 
        
        # unlocked door without key, unlock door without facing it, unlock door while door is open
        elif (key_status == 0 or not(door_ahead) or door_status == 1) and act == UD:
            cost = highCost 
        
        # move into key/door cell without picking up/toggling it
        elif ((key_ahead and key_status == 0) or (door_ahead and door_status == 0)) and act == MF:
            cost = highCost 

        else:
            cost = 1 # default cost

        return cost

    
    def nextState(self, state, act, env):
        '''
        DESC: Computes next state (x_(t+1)) for a given state and input aka motion model
        INPUT: 1x5 vector containing all states, action
        OUTPUT: next state
        '''
        # Default value
        next_state = state
        
        # Initialize values
        agent_pos = np.array([state[0], state[1]])  # agent position in grid
        agent_dir = dir[state[2]]                   # agent pointer direction
        key_status = state[3]                       # key status
        door_status = state[4]                      # door status

        front_cell = agent_pos + agent_dir          #calculate front cell

        # Initialize values for conditional checks ahead
        try:
            wall_ahead = isinstance(env.grid.get(front_cell[0], front_cell[1]), gym_minigrid.minigrid.Wall)
        except AssertionError:
            wall_ahead = True                       # check if wall ahead, treat out of map as wall
        
        try:
            key_ahead = isinstance(env.grid.get(front_cell[0], front_cell[1]), gym_minigrid.minigrid.Key)
        except AssertionError:
            key_ahead = False                       # check if key ahead
        
        try:
            door_ahead = isinstance(env.grid.get(front_cell[0], front_cell[1]), gym_minigrid.minigrid.Door)
        except AssertionError:
            door_ahead = False                      # check if door ahead
        
        try:
            at_goal = isinstance(env.grid.get(agent_pos[0], agent_pos[1]), gym_minigrid.minigrid.Goal)
        except AssertionError:
            at_goal = False                         # check if at goal
        
        
        # Conditions for stage cost
        
        # don't do anything if at goal
        if at_goal:
            next_state = state 
        
        # avoid going into walls
        elif wall_ahead and act == MF:
            next_state = state
        
        # picked key without facing it, picked key while having key
        elif (not(key_ahead) or key_status == 1) and act == PK:
            next_state = state 
        
        # unlocked door without key, unlock door without facing it, unlock door while door is open
        elif (key_status == 0 or not(door_ahead) or door_status == 1) and act == UD:
            next_state = state 
        
        # move into key/door cell without picking up/toggling it
        elif ((key_ahead and key_status == 0) or (door_ahead and door_status == 0)) and act == MF:
            next_state = state 
        
        # PK
        elif act == PK:
            next_state[3] = 1                       # pick key action
        
        # UD
        elif act == UD:
            next_state[4] = 1                       # unlock door action

        # TR
        elif act == TR:
            if next_state[2] == 3:                  # if dir == 3, then TR -> 0, else dir += 1
                next_state[2] = 0
            else:
                next_state[2] += 1 
        
        # TL
        elif act == TL:
            if next_state[2] == 0:                  # if dir == 0, then TL -> 3, else dir -= 1
                next_state[2] = 3
            else:
                next_state[2] -= 1 
        
        # MF
        elif act == MF:
            next_state[0:2] = front_cell            # move forward


        return next_state



    def doorkey_problem(self, env, info):
        '''
        DESC: Returns the optimal control sequence for a given environment 
        INPUT: environment, info
        OUTPUT: optimal control sequence
        '''

        # actions mapping
        actions = {
            0: env.actions.forward,
            1: env.actions.left,
            2: env.actions.right,
            3: env.actions.pickup,
            4: env.actions.toggle
        }


        time_horizon = 50 # set total run time, loop can break before that

        # Create all possible state space (X) combinations
        # agent_x * agent_y * agent_dir * key_stat * door_stat
        X = np.array(list(product(np.arange(info['width']), np.arange(info['height']), 
                                    [0, 1, 2, 3], [0, 1], [0, 1])))

        N_X = X.shape[0]                # no. of unique states
        policy = np.zeros(N_X)          # init policy to 0 for all states

        # create reverse mapping dictionary, which helps in state reverse lookup
        inv_X = {}
        for i in range(N_X):
            key = ''.join(map(str,X[i,:]))
            inv_X[key] = i

        V_previous = self.initvalueFunc(X.copy(), info['goal_pos'])      # init Value function (V_T)
        V = np.zeros(N_X)                                           # V_(T-1)

        # iterate over a large time horizon; terminate when V reaches steady state
        for i in tqdm(range(time_horizon)):

            
            # Iterate over all possible state space
            for j in range(N_X):
                cur_state = X[j,:]

                if np.all(cur_state[:2] == info['goal_pos']):   # find states with goal position
                    V[j] = 0                                    # goal positions have zero cost
                
                else: 
                    # Iterate over all actions to find minV
                    Q = np.zeros((5))
                    for act in [MF, TL, TR, PK, UD]:
                        
                        new_state = self.nextState(cur_state.copy(), act, env)    #x_T
                        key = ''.join(map(str,new_state))
                        Q[act] = np.min([self.stageCost(cur_state.copy(), act, env) + V_previous[inv_X[key]], 
                                        highCost])
                    
                    V[j], policy[j] = np.min(Q), np.argmin(Q)       #save the minimum cost and input

            # terminate calculation once we reach steady state
            if np.all(V == V_previous):
                print(i)
                break

            else:
                V_previous = V.copy()


        # Loop to move inside the environment and collect all optimal input sequence

        done = False            # goal reached = False
        optim_act_seq = []      # Initialize with empty sequence
        while not(done):
            # Initialize all required variables to initialize state 
            agent_pos = env.agent_pos
            door = env.grid.get(info['door_pos'][0], info['door_pos'][1])
            is_open = door.is_open
            is_carrying = env.carrying is not None
            
            # define current state based on the current environment
            cur_state = np.array([agent_pos[0], agent_pos[1], env.agent_dir, is_carrying, is_open])

            # find index of state, same as of the current state -> use the corresponding policy
            key = ''.join(map(str,cur_state))
            _, _, done, _ = env.step(actions[policy[inv_X[key]]])
            
            optim_act_seq.append(policy[inv_X[key]])

        return optim_act_seq



class problem_B():
    def initvalueFunc(self, states):
        '''
        DESC: Initializes the value function for x_T; highCost everywhere and 0 at the goals
        INPUT: 1x8 vector containing all states
        OUTPUT: V_T (initial value function)
        '''
        V = np.zeros(states.shape[0]) + highCost # initialize V with very high cost

        # find indices of goal positions and goal states, for which V = 0
        inGoal1 = np.logical_and(np.all(states[:,:2] == goal_loc[0], axis=1), states[:,7] == 0)
        inGoal2 = np.logical_and(np.all(states[:,:2] == goal_loc[1], axis=1), states[:,7] == 1)
        inGoal3 = np.logical_and(np.all(states[:,:2] == goal_loc[2], axis=1), states[:,7] == 2)

        idx = np.logical_or(np.logical_or(inGoal1,inGoal2), inGoal3)  # find indexes of goal positions    
        V[idx] = 0                  # goal positions have zero cost
        return V

    def checkIfInGoal(self, state):
        '''
        DESC: Checks if a current state is in a goal position
        INPUT: 1x8 vector containing all states
        OUTPUT: True/False
        '''

        # check if current state is a goal position or not
        inGoal1 = np.logical_and(np.all(state[:2] == goal_loc[0]), state[7] == 0)
        inGoal2 = np.logical_and(np.all(state[:2] == goal_loc[1]), state[7] == 1)
        inGoal3 = np.logical_and(np.all(state[:2] == goal_loc[2]), state[7] == 2)

        idx = np.logical_or(np.logical_or(inGoal1,inGoal2), inGoal3)  # find indexes of goal positions   
        
        return np.any(idx)

    def stageCost(self, state, act):
        '''
        DESC: Computes cost for a given state and input
        INPUT: 1x5 vector containing all states, action
        OUTPUT: cost
        '''

        # Initialize values
        agent_pos = np.array([state[0], state[1]])  # agent position in grid
        agent_dir = dir[state[2]]                   # agent pointer direction
        key_status = state[3]                       # key status
        door1_status = state[4]                     # door1 status
        door2_status = state[5]                     # door2 status
        key_location = state[6]                     # key location
        goal_location = state[7]                    # goal location

        front_cell = agent_pos + agent_dir          #calculate front cell

        # Initialize values for conditional checks ahead
        wall_ahead = front_cell[0] == 0  or front_cell[0] == 7 or front_cell[1] == 0 or front_cell[1] == 7 or np.all(front_cell == [4,1]) or \
                np.all(front_cell == [4,3]) or np.all(front_cell == [4,4]) or np.all(front_cell == [4,6])           # check if wall ahead, treat out of map as wall
        
        key_ahead = np.logical_or(np.logical_or(np.all(front_cell == key_loc[0]) and key_location == 0, 
                                    np.all(front_cell == key_loc[1]) and key_location == 1), 
                                np.all(front_cell == key_loc[2]) and key_location == 2)                             # check if key ahead
        
        door1_ahead = np.all(front_cell == [4,2])                                                                   # check if door1 ahead
        door2_ahead = np.all(front_cell == [4,5])                                                                   # check if door2 ahead
        
        at_goal = self.checkIfInGoal(state.copy())                                                                       # check if at goal

        
        # Conditions for stage cost

        # don't do anything if at goal
        if at_goal:
            cost = 0
            
        # avoid going into walls   
        elif (wall_ahead or front_cell[0] < 0 or front_cell[0] > 7 or front_cell[1] < 0 or front_cell[1] > 7) and act == MF:
            cost = highCost 
        
        # picked key without facing it, picked key while having key
        elif (not(key_ahead) or key_status == 1) and act == PK:
            cost = highCost 
        
        # unlocked door without key, unlock door without facing it, unlock door while door is open
        elif (key_status == 0 or not(door1_ahead or door2_ahead) or (door1_status == 1 and door1_ahead) or (door2_status == 1 and door2_ahead)) and act == UD:
            cost = highCost 
        
        # move into key/door cell without picking up/toggling it
        elif ((key_ahead and key_status == 0) or (door1_ahead and door1_status == 0) or (door2_ahead and door2_status == 0)) and act == MF:
            cost = highCost 

        else:
            cost = 1 # default cost

        return cost

    def nextState(self, state, act):
        '''
        DESC: Computes next state (x_(t+1)) for a given state and input aka motion model
        INPUT: 1x5 vector containing all states, action
        OUTPUT: next state
        '''
        # Default value
        next_state = state
        
        # Initialize values
        agent_pos = np.array([state[0], state[1]])  # agent position in grid
        agent_dir = dir[state[2]]                   # agent pointer direction
        key_status = state[3]                       # key status
        door1_status = state[4]                     # door1 status
        door2_status = state[5]                     # door2 status
        key_location = state[6]                     # key location
        goal_location = state[7]                    # goal location

        front_cell = agent_pos + agent_dir          #calculate front cell

        # Initialize values for conditional checks ahead
        wall_ahead = front_cell[0] == 0  or front_cell[0] == 7 or front_cell[1] == 0 or front_cell[1] == 7 or np.all(front_cell == [4,1]) or \
                np.all(front_cell == [4,3]) or np.all(front_cell == [4,4]) or np.all(front_cell == [4,6])           # check if wall ahead, treat out of map as wall
        
        key_ahead = np.logical_or(np.logical_or(np.all(front_cell == key_loc[0]) and key_location == 0, 
                                    np.all(front_cell == key_loc[1]) and key_location == 1), 
                                np.all(front_cell == key_loc[2]) and key_location == 2)                             # check if key ahead
        
        door1_ahead = np.all(front_cell == [4,2])                                                                   # check if door1 ahead
        door2_ahead = np.all(front_cell == [4,5])                                                                   # check if door2 ahead
        
        at_goal = self.checkIfInGoal(state.copy())                                                                       # check if at goal
        
        
        # Conditions for stage cost
        
        # don't do anything if at goal
        if at_goal:
            next_state = state 
        
        # avoid going into walls
        elif (wall_ahead or front_cell[0] < 0 or front_cell[0] > 7 or front_cell[1] < 0 or front_cell[1] > 7) and act == MF:
            next_state = state
        
        # picked key without facing it, picked key while having key
        elif (not(key_ahead) or key_status == 1) and act == PK:
            next_state = state 
        
        # unlocked door without key, unlock door without facing it, unlock door while door is open
        elif (key_status == 0 or not(door1_ahead or door2_ahead) or (door1_status == 1 and door1_ahead) or (door2_status == 1 and door2_ahead)) and act == UD:
            next_state = state 
        
        # move into key/door cell without picking up/toggling it
        elif ((key_ahead and key_status == 0) or (door1_ahead and door1_status == 0) or (door2_ahead and door2_status == 0)) and act == MF:
            next_state = state 
        
        # PK
        elif act == PK:
            next_state[3] = 1                       # pick key action
        
        # UD
        elif act == UD and door1_ahead:
            next_state[4] = 1                       # unlock door1 action
        
        elif act == UD and door2_ahead:
            next_state[5] = 1                       # unlock door1 action

        # TR
        elif act == TR:
            if next_state[2] == 3:                  # if dir == 3, then TR -> 0, else dir += 1
                next_state[2] = 0
            else:
                next_state[2] += 1 
        
        # TL
        elif act == TL:
            if next_state[2] == 0:                  # if dir == 0, then TL -> 3, else dir -= 1
                next_state[2] = 3
            else:
                next_state[2] -= 1 
        
        # MF
        elif act == MF:
            next_state[0:2] = front_cell            # move forward


        return next_state
    
    def generate_policy(self):
        '''
        DESC: Generates single control policy for complete state space
        INPUT: -
        OUTPUT: complete policy
        '''
        time_horizon = 50 # set total run time, loop can break before that
 
        # Create all possible state space (X) combinations
        # agent_x * agent_y * agent_dir * key_stat * door_1_stat * door_2_stat * key_loc * goal_loc
        X = np.array(list(product(np.arange(8), np.arange(8), [0, 1, 2, 3], [0, 1], [0, 1], [0, 1], [0, 1, 2], [0, 1, 2])))

        N_X = X.shape[0]                # no. of unique states
        policy = np.zeros(N_X)          # init policy to 0 for all states

        # create reverse mapping dictionary
        inv_X = {}
        for i in range(N_X):
            key = ''.join(map(str,X[i,:]))
            inv_X[key] = i
        
        
        V_previous = self.initvalueFunc(X.copy())      # init Value function (V_T)
        V = np.zeros(N_X)                                           # V_(T-1)



        # iterate over a large time horizon; terminate when V reaches steady state
        for i in tqdm(range(time_horizon)):

            
            # Iterate over all possible state space
            for j in range(N_X):
                cur_state = X[j,:] 

                if self.checkIfInGoal(cur_state.copy()):             # find states with goal position
                    V[j] = 0                                    # goal positions have zero cost
                
                else: 
                    # Iterate over all actions to find minV
                    Q = np.zeros((5))
                    for act in [MF, TL, TR, PK, UD]:
                        
                        new_state = self.nextState(cur_state.copy(), act)    #x_T
                        key = ''.join(map(str,new_state))
                        Q[act] = np.min([self.stageCost(cur_state.copy(), act) + V_previous[inv_X[key]], 
                                            highCost])
                    
                    V[j], policy[j] = np.min(Q), np.argmin(Q)   #save the minimum cost and input

            # terminate calculation once we reach steady state
            if np.all(V == V_previous):
                print(i)        #calculate no of time steps for convergence
                break

            else:
                print(np.linalg.norm(V_previous - V)) # uncomment/comment if want to check whether V is converging to steady state
                V_previous = V.copy()

        return policy

    def run_saved_policy(self, env, info, policy):
        '''
        DESC: Runs saved policy to generate gif
        INPUT: environment, env info, complete policy
        OUTPUT: optimal sequence
        '''

        # actions mapping
        actions = {
            0: env.actions.forward,
            1: env.actions.left,
            2: env.actions.right,
            3: env.actions.pickup,
            4: env.actions.toggle
        }

        # Create all possible state space (X) combinations
        # agent_x * agent_y * agent_dir * key_stat * door_1_stat * door_2_stat * key_loc * goal_loc
        X = np.array(list(product(np.arange(8), np.arange(8), [0, 1, 2, 3], [0, 1], [0, 1], [0, 1], [0, 1, 2], [0, 1, 2])))

        N_X = X.shape[0]                # no. of unique states

        # create reverse mapping dictionary
        inv_X = {}
        for i in range(N_X):
            key = ''.join(map(str,X[i,:]))
            inv_X[key] = i

        
        done = False
        optim_act_seq = []
        while not(done):
            # Initialize all required variables to initialize state 
            agent_pos = env.agent_pos
            door1 = env.grid.get(4,2)                   # door 1 data
            door2 = env.grid.get(4,5)                   # door 2 data
            is_1_open = door1.is_open                   # door 1 status
            is_2_open = door2.is_open                   # door 2 status
            is_carrying = env.carrying is not None      # key status
            
            # convert to dict key format
            key_loc = ''.join(map(str,info['key_pos']))
            goal_loc = ''.join(map(str,info['goal_pos']))
            
            # define current state based on the current environment
            cur_state = np.array([agent_pos[0], agent_pos[1], env.agent_dir, is_carrying, is_1_open, is_2_open, key_loc_inv[key_loc], goal_loc_inv[goal_loc]])

            # find index of state, same as of the current state -> use the corresponding policy
            key = ''.join(map(str,cur_state))
            _, _, done, _ = env.step(actions[policy[inv_X[key]]])
            
            optim_act_seq.append(policy[inv_X[key]])        #append optimal sequence for particular env

        return optim_act_seq




def partA():
    '''
    DESC: Calls required functions and returns values/gifs to complete part a
    '''
    doorkey = problem_A()

    env_path = env_dict[5]                              # choose any particular environment
    env, info = load_env(env_path)                      # load an environment
    seq = doorkey.doorkey_problem(env, info)            # find the optimal action sequence
    
    gif_path = './gif/' + env_path[7:-4] + '.gif'
    draw_gif_from_seq(seq, load_env(env_path)[0], gif_path) # draw a GIF & save
    print(seq)

    
def partB():
    '''
    DESC: Calls required functions and returns values/gifs to complete part b
    '''
    random_doorkey = problem_B()

    '''Load defined environment; uncomment out if want to load SPECIFIC environment'''
    env_path = './envs/random_envs/DoorKey-8x8_36.pickle'
    env, info = load_env(env_path)  

    '''load random environment; uncomment out if want to load RANDOM environment'''
    # env_folder = './envs/random_envs'
    # env, info, env_path = load_random_env(env_folder)       
    
    '''generate and save complete policy; uncomment only if you want to REGENERATE policy'''
    # control_policy = random_doorkey.generate_policy() 
    # np.save('./gif/policy_part_b.npy', control_policy)

    '''run random environment with pre-computed policy; uncomment only if you want to TEST policy'''
    control_policy = np.load('./gif/policy_part_b.npy')                 # load pre-computed policy
    seq = random_doorkey.run_saved_policy(env, info, control_policy)    # obtain sequence of inputs for the given map to generate gif
    gif_path = './gif/' + env_path[19:-7] + '.gif'
    draw_gif_from_seq(seq, load_env(env_path)[0], gif_path) # draw a GIF & save


if __name__ == '__main__':
    
    '''Uncomment whichever part is to be run'''
    # partA()
    # partB()
