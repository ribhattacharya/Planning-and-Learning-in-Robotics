from os import terminal_size
from time import time
import numpy as np
from utils import visualize
from casadi import *

# Simulation params
np.random.seed(10)
time_step = 0.5 # time between steps in seconds
sim_time = 120    # simulation time

# Car params
x_init = 1.5
y_init = 0.0
theta_init = np.pi/2


class CEC():
    """
    DESC: Contains methods required to implement the CEC controller
    """

    def __init__(self) -> None:
        """
        DESC:   Initialize all parameters needed for the simulation. 
                Choose obstacle yes/no from here, and edit any 
                parameters if required.
        """
        # Initialize constraint values, to be used in program later
        self.x_min, self.x_max = -3, 3
        self.y_min, self.y_max = -3, 3
        self.pi_min, self.pi_max = -np.pi, np.pi
        self.v_min, self.v_max = 0, 1
        self.w_min, self.w_max = -1, 1

        # Change to True/False as per requirement
        self.sim_with_obstacles = False      # simulate with/without obstacles?
        self.motion_noise = True            # noise in car motion model?
        self.save_gifs = True              # save gifs?

        # Initialize nlp solver parameters
        if self.sim_with_obstacles:
            self.nlp_params = {
                'N': 10, 
                'Q': 0.7 * MX.eye(2), 
                'R': 1 * MX.eye(2), 
                'q': 10, 
                'gamma':0.87}
        else:
            self.nlp_params = {
                'N': 10, 
                'Q': 1 * MX.eye(2), 
                'R': 0.1 * MX.eye(2), 
                'q': 10, 
                'gamma':0.87}


    # This function implements error state dynamics
    def carErrorNextState(self, k, e_t, control):
        """
        DESC:   Computes the next error state for the car according to error dynamics model
        INPUT:  Current iteration, current error state, control input
        OUTPUT: Next error state computed according to error dynamics model
        """
        # Collect current and next reference trajectory data, calculate delta
        cur_ref = np.array(lissajous(k))
        next_ref = np.array(lissajous(k + 1))
        del_ref = cur_ref - next_ref

        
        # rotation matrix
        angle = e_t[2] +  cur_ref[2]    
        rot_3d_z = vertcat( hcat([cos(angle), 0]), 
                            hcat([sin(angle), 0]), 
                            hcat([0, 1])
                        )
        f = rot_3d_z @ control
        
        e_t_plus_one = e_t + time_step * f + del_ref    # compute next error state

        return e_t_plus_one
    
    def cecController(self, cur_iter, cur_err_state):
        """
        DESC:   Computes the next (suboptimal) control input based on the current error state
        INPUT:  Current iteration, current error state
        OUTPUT: Control input calculated according to the CEC
        """

        N = self.nlp_params['N']       # Time steps to look ahead for computing value function
        
        # Assign costs/weights
        Q = self.nlp_params['Q'] 
        R = self.nlp_params['R'] 
        q = self.nlp_params['q'] 
        gamma = self.nlp_params['gamma'] 

        # BEGIN OPTIMIZATION PROBLEM
        opti = Opti()
        
        # OPTIMIZATION VARIABLES 
        E = opti.variable(3,N+1) # error states
        U = opti.variable(2,N)   # control

        # OBJECTIVE FUNCTION (creation loop)
        objective = 0
        for i in range(N):

            p_t     = E[:2,i]
            theta_t = E[2,i]
            control = U[:,i]
            
            cur_ref = lissajous(cur_iter + i)   # current reference trajectory state
            actual_x = E[0,i] + cur_ref[0]      # actual state with errors
            actual_y = E[1,i] + cur_ref[1]
            
            
            # --------------------- Add constraints for each state at each time-------------------

            # Obstacle/collision constraints (for circles; added tolerance of 0.05 in the radius)
            if self.sim_with_obstacles:
                opti.subject_to((actual_x + 2)**2 + (actual_y + 2)**2 >= 0.5**2 + 0.05)
                opti.subject_to((actual_x - 1)**2 + (actual_y - 2)**2 >= 0.5**2 + 0.05)
                

            # Map bound constraints
            opti.subject_to(opti.bounded(self.x_min, actual_x, self.x_max))
            opti.subject_to(opti.bounded(self.y_min, actual_y, self.y_max))
            # opti.subject_to(opti.bounded(self.pi_min, E[2,i] + cur_ref[2], self.pi_max))
            
            
            # Motion model constraint
            opti.subject_to(E[:,i+1] == self.carErrorNextState(cur_iter + i, E[:,i], U[:,i]))


            # add terms to objective symbolically
            objective += gamma**i * (p_t.T @ Q @ p_t + q*((1 - cos(theta_t))**2) + control.T @ R @ control)
        
        # ADD TERMINAL COST
        terminal_cost = gamma**N * (E[:2,-1].T @ Q @ E[:2,-1] + q*((1 - cos(E[2,-1]))**2) + U[:,-1].T @ R @ U[:,-1])
        objective += terminal_cost


        # MORE CONSTRAINTS
        opti.subject_to(opti.bounded(self.v_min,  U[0,:], self.v_max))  # linear velocity constraints
        opti.subject_to(opti.bounded(self.w_min,  U[1,:], self.w_max))  # angular velocity constraints
        opti.subject_to(E[:,0] ==  cur_err_state)                       # Initial error state


        # SOLVER INITIAL VALUES (GUESS)
        opti.set_initial(E[:,0], cur_err_state)
        opti.set_initial(U[:,0], [0.5, -0.25])

        
        # OBJECTIVE (MINIMIZATION)
        opti.minimize(objective)


        # SOLVE NLP
        opts = {'ipopt.print_level':0, 'print_time':0}  # suppress solver outputs
        opti.solver("ipopt", opts) # set solver type and parameters
        sol = opti.solve()   
        
        return sol.value(U[:,0])    # return first move from the list of suboptimal moves 


# This function returns the reference point at time step k
def lissajous(k):
    xref_start = 0
    yref_start = 0
    A = 2
    B = 2
    a = 2*np.pi/50
    b = 3*a
    T = np.round(2*np.pi/(a*time_step))
    k = k % T
    delta = np.pi/2
    xref = xref_start + A*np.sin(a*k*time_step + delta)
    yref = yref_start + B*np.sin(b*k*time_step)
    v = [A*a*np.cos(a*k*time_step + delta), B*b*np.cos(b*k*time_step)]
    thetaref = np.arctan2(v[1], v[0])
    return [xref, yref, thetaref]



# This function implement the car dynamics
def car_next_state(time_step, cur_state, control, noise = True):
    theta = cur_state[2]
    rot_3d_z = np.array([[np.cos(theta), 0], [np.sin(theta), 0], [0, 1]])
    f = rot_3d_z @ control
    mu, sigma = 0, 0.04 # mean and standard deviation for (x,y)
    w_xy = np.random.normal(mu, sigma, 2)
    mu, sigma = 0, 0.004  # mean and standard deviation for theta
    w_theta = np.random.normal(mu, sigma, 1)
    w = np.concatenate((w_xy, w_theta))
    if noise:
        return cur_state + time_step * f.flatten() + w
    else:
        return cur_state + time_step * f.flatten()



if __name__ == '__main__':
    # Obstacles in the environment
    obstacles = np.array([[-2,-2,0.5], [1,2,0.5]])
    # Params
    traj = lissajous
    ref_traj = []
    error = 0.0
    car_states = []
    times = []
    # Start main loop
    main_loop = time()  # return time in sec
    # Initialize state and error state
    cur_state = np.array([x_init, y_init, theta_init])
    cur_iter = 0
    error_state_list = []

    # Cretae controller object
    cec_control = CEC()

    # Main loop
    while (cur_iter * time_step < sim_time):
        t1 = time()
        # Get reference state
        cur_time = cur_iter * time_step
        cur_ref = traj(cur_iter)
        # Save current state and reference state for visualization
        ref_traj.append(cur_ref)
        car_states.append(cur_state)

        ################################################################
        # Generate control input
        error_state = cur_state - cur_ref
        control = cec_control.cecController(cur_iter, error_state.copy())
        print("[v,w]", control)
        ################################################################

        # Apply control input
        next_state = car_next_state(time_step, cur_state, control, noise=cec_control.motion_noise)
        # Update current state
        cur_state = next_state
        # Loop time
        t2 = time()
        print(cur_iter)
        print(t2-t1)
        times.append(t2-t1)
        error = error + np.linalg.norm(cur_state - cur_ref)
        cur_iter = cur_iter + 1

    main_loop_time = time()
    print('\n\n')
    print('Total time: ', main_loop_time - main_loop)
    print('Average iteration time: ', np.array(times).mean() * 1000, 'ms')
    print('Final error: ', error)

    # Visualization
    ref_traj = np.array(ref_traj)
    car_states = np.array(car_states)
    times = np.array(times)
    visualize(car_states, ref_traj, obstacles, times, time_step, save=cec_control.save_gifs)