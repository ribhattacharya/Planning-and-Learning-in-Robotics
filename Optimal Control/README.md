# Infinite-Horizon Stochastic Optimal Control
- _(Cmd+Shift+V to go into markdown reader mode in Mac)_
- _(Ctrl+Shift+V to go into markdown reader mode in Windows)_


There are 2 files,
1. **main.py**: implements the environment and contains the CEC controller
2. **utils.py**: contains methods for animations/visualizations which are called from _main.py_


# **main.py**
Contains the complete implementation of the envionment and the CEC controller. (Only edited/newly added functions have been described here).

## class **CEC**
Used to describe the CEC() object, that contains parameters required to intialize the controller and methods for implementing the same

### 1. function **init(self)**
Initialize all the variables/parameters that are required by the CEC controller implementation like 
- map bounds and optimization variable bounds
- boolean (T/F) simulation parameters (simulate with/without obstacles, simulate with/without noise, save/not save gifs)
- NLP solver parameters (Q,R,q,T, gamma).

### 2. function **carErrorNextState**
- DESC:   Computes the next error state for the car according to error dynamics model
- INPUT:  Current iteration, current error state, control input
- OUTPUT: Next error state (e_{t+1}) computed according to error dynamics model

### 3. function **cecController**
- DESC:   Computes the next (suboptimal) control input (u) based on the current error state (e_{t})
- INPUT:  Current iteration, current error state
- OUTPUT: Control input calculated according to the CEC


