import math
import serial
import time
import RPi.GPIO as GPIO
import pickle

from IK_func_8 import IK, RSF , RSB , REF , REB , LSF , LEB , LEF , LEB , theta2servo , trajectory_plan , traj_use , turn_angles

def save_data(filename, data):
    """
    Saves the provided data into a pickle file for quick loading.
    
    Args:
    filename (str): The name of the file to save the data.
    data (any): The data to be saved.
    """
    with open(filename, 'wb') as f:
        pickle.dump(data, f)
        
theta1_stand , theta2_stand = 50 , 125
theta1_down , theta2_down = 10 , 70
stand = theta2servo(theta1_stand , theta2_stand ,theta1_stand , theta2_stand, theta1_stand , theta2_stand, theta1_stand , theta2_stand)

# Setup trajectory parameters
x0 , y0 = -20 , 180  # coordinates for the starting point of the leg
R = 22 # radius of step's circle
steps = 10 # number of iterations for a step
''' working sets:
slowmotion fot showing the consept: R , steps , N_steps , t_delay_steps = 25 , 20 , 5 , 0.2
best motion: R , steps , N_steps , t_delay_steps = 20 , 10 , 5 , 0.2
'''
N_steps = 5  # number of whole steps
x_trj , y_trj , len_arc = trajectory_plan(x0 , y0 , R , steps)
step_ang_mat = traj_use(x_trj, y_trj, R, steps)
save_data("step_angles.pkl", step_ang_mat)


    
r_turn = 25
n_turnsteps = 10
x_turn , y_turn , b = trajectory_plan(x0 , y0 , r_turn , n_turnsteps)
turn_ang_mat = traj_use(x_turn, y_turn, r_turn, n_turnsteps)
print(turn_ang_mat)

R_turn , L_turn = turn_angles(turn_ang_mat,stand)
save_data("turn_angles.pkl", (R_turn, L_turn))

walk_back = [[0 for _ in range(8)] for _ in range(n_turnsteps )]
for ind in range(n_turnsteps ):
    walk_back[ind] = turn_ang_mat[n_turnsteps -1-ind]
save_data("walkback_angles.pkl", walk_back)
