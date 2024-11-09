import math
import serial
import time
import RPi.GPIO as GPIO
import pickle
import os

from IK_func_8 import IK, RSF , RSB , REF , REB , LSF , LEB , LEF , LEB , theta2servo , trajectory_plan , traj_use , turn_angles

def ledon(ledpin):
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(ledpin , GPIO.OUT)
    GPIO.output(ledpin , GPIO.HIGH)
    time.sleep(2)
    GPIO.output(ledpin , GPIO.LOW)
print('initializing')
# Setup serial communication and transmission function
ser = serial.Serial('/dev/ttyAMA0',115200, timeout=1)
t_delay_poses = 1.5
def send_angles(angles, pose_name, slp = t_delay_poses):
    print(pose_name)
    if len(angles) != 8:
        print('error')
        return
    hex_array = bytearray(angles)
    ser.write(hex_array)
    time.sleep(slp)
    print('done ' ,angles , pose_name)

def load_data(filename):
    """
    Loads data from a pickle file.
    
    Args:
    filename (str): The name of the file to load the data from.

    Returns:
    data (any): The loaded data.
    """
    with open(filename, 'rb') as f:
        return pickle.load(f)

# def executing function for steps
def step_act(step_angles, N , delay):
    send_angles(stand , 'stand')
    for n in range(N):
        print(f"iteration {n}")
        for m in range(len(step_angles)):
            send_angles(step_angles[m] , f"point {m}" , t_delay_step)
    send_angles(stand , 'stand')

def turn(turn_angles, N, delay ):	
    send_angles(stand , 'stand')
    n_angles = len(turn_angles)
    for n in range(N):
        #print(f"iteration {n}")
        for m in range(n_angles):
            send_angles(turn_angles[m] + stand[2:-1] , f"point {m}" , t_delay_step)
    send_angles(stand , 'stand')


# Set basic positions
theta1_stand , theta2_stand = 50 , 125
theta1_down , theta2_down = 10 , 70

down_showoff = [10 , 70 , 10 , 70 , 165 , 170 , 165 , 170]    
stand_showoff = [50, 160, 50, 160, 125, 80, 125, 80]  # motor 6 is not synced to motor 8's zero point because of mechanical damage
sit_showoff = [50 , 160 , 10 , 70 , 125 , 80 , 165 , 170]
zeros = [0,90,0,70,175,150,175,170]
down = theta2servo(theta1_down , theta2_down ,theta1_down , theta2_down, theta1_down , theta2_down, theta1_down , theta2_down)
stand = theta2servo(theta1_stand , theta2_stand ,theta1_stand , theta2_stand, theta1_stand , theta2_stand, theta1_stand , theta2_stand)
sit = theta2servo(theta1_stand , theta2_stand ,theta1_down , theta2_down, theta1_stand , theta2_stand, theta1_down , theta2_down)
print('calculating Inverse Kinematics and Trajectory')
# Setup trajectory parameters
#x0 , y0 = -20 , 180  # coordinates for the starting point of the leg
#R = 20 # radius of step's circle
#steps = 10 # number of iterations for a step
#N_steps = 5  # number of whole steps
#t_delay_step = 0.2


''' working sets:
slowmotion fot showing the consept: R , steps , N_steps , t_delay_steps = 25 , 20 , 5 , 0.2
best motion: R , steps , N_steps , t_delay_steps = 20 , 10 , 5 , 0.2
'''


# set up main loop
# Set up GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
sw_pin = 4
# Set up GPIO 17 as an input with pull-down resistor
GPIO.setup(sw_pin, GPIO.IN, GPIO.PUD_DOWN)
# get the trajectory data
#x_trj , y_trj , len_arc = trajectory_plan(x0 , y0 , R , steps)

print('waiting for switch changing')
switch_on = False
while not switch_on:
	ledon(26)
	button_state = GPIO.input(sw_pin)
	if button_state == GPIO.HIGH:
		print(type(button_state), GPIO.HIGH)
		print('high')
		switch_on = True
	else:
		print(button_state)
		print('low')
	time.sleep(0.5)

# show poses
send_angles(down_showoff, 'down')
send_angles(stand_showoff , 'stand')
send_angles(sit_showoff, 'sit')
send_angles(down_showoff, 'down')
send_angles(sit_showoff, 'sit')
send_angles(stand_showoff, 'stand')
#send_angles(zeros, 'zero')

os.chdir('/usr/local/bin')
step_ang_mat = load_data("step_angles.pkl")
R_turn, L_turn = load_data("turn_angles.pkl")
walk_back= load_data("walkback_angles.pkl")

ledon(26)

# step action
N_steps = 5  # number of whole steps
t_delay_step = 0.2
send_angles(stand, 'stand')
step_act(step_ang_mat, N_steps , t_delay_step)

#turn right
N_turn_iter = 4

step_act(R_turn, N_turn_iter, t_delay_step*2)
#turn left
#step_act(L_turn, N_turn_iter , t_delay_step*2)

step_act(step_ang_mat, N_steps , t_delay_step)
step_act(walk_back, N_steps , t_delay_step)


#end showoff
send_angles(stand_showoff , 'stand',t_delay_poses*2)
send_angles(sit_showoff, 'sit',t_delay_poses*2)
send_angles(down_showoff, 'down',t_delay_poses*2)

