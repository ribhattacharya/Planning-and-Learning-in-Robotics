# Dynamic Programming: Door-Key problem
(Cmd+Shift+V to go into markdown reader mode in Mac)
(Ctrl+Shift+V to go into markdown reader mode in Windows)


There are 2 files,
1. **doorkey.py**: Part a & b
2. **policy_part_b.npy**: Pre computed policy for all states; reqd. for Part b)


# **doorkey.py** 
script is organized in following 5 parts;

## 1. class **problem_A** 
- **initvalueFunc**: Initializes the value function for x_T; highCost everywhere and 0 at the goals
- **stageCost**: Computes cost for a given state and input
- **nextState**: Computes next state (x_(t+1)) for a given state and input aka motion model
- **doorkey_problem**: Returns the optimal control sequence for a given environment 

## 2. class **problem_B**
- **initvalueFunc**: Initializes the value function for x_T; highCost everywhere and 0 at the goals
- **checkIfInGoal**: Checks if a current state is in a goal position
- **stageCost**: Computes cost for a given state and input
- **nextState**: Computes next state (x_(t+1)) for a given state and input aka motion model
- **generate_policy**: Generates single control policy for complete state space
- **run_saved_policy**: Runs saved policy to generate gif


## 3. **part_A**
Calls required functions and returns values/gifs to complete part a

## 4. **part_B**
Calls required functions and returns values/gifs to complete part b. You can choose amongst the following by commenting/uncommenting lines.
- Loading specific environments vs loading random environments
- Regenerate control policy again or just run the policy over an environment

Remember to change the location of the **policy_part_b.npy** file in this function, if running on a local machine.

## 5. **if __name__ == '__main__'**
Calls the coressponding problem parts a & b

# Results

|                                     |                                        |                                        |                                        |                                        |
|-------------------------------------|----------------------------------------|----------------------------------------|----------------------------------------|----------------------------------------|
| ![best](gif/doorkey-5x5-normal.gif) | ![bestvid](gif/doorkey-6x6-direct.gif) | ![bestvid](gif/doorkey-6x6-normal.gif) |![bestvid](gif/doorkey-6x6-shortcut.gif)|![bestvid](gif/doorkey-6x6-shortcut.gif)|
| ![best](gif/DoorKey-8x8_01.gif)     | ![bestvid](gif/DoorKey-8x8_02.gif)     | ![bestvid](gif/DoorKey-8x8_03.gif)     |![bestvid](gif/DoorKey-8x8_04.gif)      |![bestvid](gif/DoorKey-8x8_05.gif)      |
| ![best](gif/DoorKey-8x8_06.gif)     | ![bestvid](gif/DoorKey-8x8_07.gif)     | ![bestvid](gif/DoorKey-8x8_08.gif)     |![bestvid](gif/DoorKey-8x8_09.gif)      |![bestvid](gif/DoorKey-8x8_10.gif)      |
| ![best](gif/DoorKey-8x8_11.gif)     | ![bestvid](gif/DoorKey-8x8_12.gif)     | ![bestvid](gif/DoorKey-8x8_13.gif)     |![bestvid](gif/DoorKey-8x8_14.gif)      |![bestvid](gif/DoorKey-8x8_15.gif)      |
| ![best](gif/DoorKey-8x8_16.gif)     | ![bestvid](gif/DoorKey-8x8_17.gif)     | ![bestvid](gif/DoorKey-8x8_18.gif)     |![bestvid](gif/DoorKey-8x8_19.gif)      |![bestvid](gif/DoorKey-8x8_20.gif)      |
| ![best](gif/DoorKey-8x8_21.gif)     | ![bestvid](gif/DoorKey-8x8_22.gif)     | ![bestvid](gif/DoorKey-8x8_23.gif)     |![bestvid](gif/DoorKey-8x8_24.gif)      |![bestvid](gif/DoorKey-8x8_25.gif)      |
| ![best](gif/DoorKey-8x8_26.gif)     | ![bestvid](gif/DoorKey-8x8_27.gif)     | ![bestvid](gif/DoorKey-8x8_28.gif)     |![bestvid](gif/DoorKey-8x8_29.gif)      |![bestvid](gif/DoorKey-8x8_30.gif)      |
| ![best](gif/DoorKey-8x8_31.gif)     | ![bestvid](gif/DoorKey-8x8_32.gif)     | ![bestvid](gif/DoorKey-8x8_33.gif)     |![bestvid](gif/DoorKey-8x8_34.gif)      |![bestvid](gif/DoorKey-8x8_35.gif)      |
| ![best](gif/DoorKey-8x8_36.gif)     | ![bestvid](gif/doorkey-8x8-direct.gif) | ![bestvid](gif/doorkey-8x8-normal.gif) |![bestvid](gif/doorkey-8x8-shortcut.gif)| |





