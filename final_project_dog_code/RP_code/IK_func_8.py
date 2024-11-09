import math
import serial
import time


def RSF(theta1):
    #deg
    if theta1 < 0:
        return 0
    elif theta1 > 90:
        return 90
    elif 0 < theta1 < 90:
        return theta1

def RSB(theta1):
    #deg
    if theta1 < 0:
        return 0
    elif theta1 > 90:
        return 90
    elif 0 < theta1 < 90:
        return theta1
def LSF(theta1):
    #deg
    theta_L = 175- theta1
    if theta_L < 90:
        return 90
    elif theta_L > 180:
        return 180
    elif 90 < theta_L < 180:
        return theta_L
def LSB(theta1):
    #deg
    theta_L = 175- theta1
    if theta_L < 90:
        return 90
    elif theta_L > 180:
        return 180
    elif 90 < theta_L < 180:
        return theta_L
def REF(theta2):
    #deg
    if theta2 < 60:
        return 60
    elif theta2 > 180:
        return 180
    elif 60 < theta2 < 180:
        return theta2
def REB(theta2):
    #deg
    theta2 = theta2 +15 
    if theta2 < 60:
        return 60
    elif theta2 > 180:
        return 180
    elif 60 < theta2 < 180:
        return theta2
def LEF(theta2):
    #deg
    theta_L = 240- theta2
    if theta_L < 60:
        return 60
    elif theta_L > 180:
        return 180
    elif 60 < theta_L < 180:
        return theta_L
def LEB(theta2):
    #deg
    theta_L = 225- theta2
    if theta_L < 60:
        return 60
    elif theta_L > 180:
        return 180
    elif 60 < theta_L < 180:
        return theta_L
def theta2servo(rsf , ref , rsb , reb , lsf , lef , lsb , leb):
    return [RSF(rsf) , REF(ref) ,RSB(rsb) , REB(reb) , LSF(lsf) , LEF(lef) , LSB(lsb) , LEB(leb)]
#example
#downn = theta2servo(10 , 70 , 10 , 70 , 10 , 70 , 10 , 70)
#standd = theta2servo(50,125,50,125,50,125,50,125)
#for i in range(8):
    #print(f"down {i+1} :  {downn[i]-down[i]}")
    #print(f"stand {i+1} :  {standd[i]-stand[i]}")


import math

def IK(x, y):
    """
    This function calculates the inverse kinematics for a robot's 2DOF leg.
    Given the target coordinates (x, y), the function returns possible solutions for
    the joint angles (theta1, theta2) in degrees and the angle alpha (in radians).
    
    Parameters:
    x (int): The x-coordinate of the target position in millimeters.
    y (int): The y-coordinate of the target position in millimeters.
    
    Returns:
    theta1_val (list of float): Possible values for the angle theta1 (in degrees).
    theta2_val (list of float or bool): Possible values for the angle theta2 (in degrees), 
                                        or False if the solution is not defined.
    alpha (float): The angle between the segments L2 and the horion (in degrees). beta = alpha + theta1
    """
    
    # Constants based on the robot's design
    gamma_deg = 20.4  # Fixed angle gamma in degrees
    gamma = math.radians(gamma_deg)  # Convert gamma to radians
    d1 = 20  # mm, distance of first joint from origin
    d2 = 90  # mm, length of the second link
    l1 = 24.2  # mm, length of the first link
    l4 = 85.5  # mm, length of the last link
    L1 = 120  # mm, distance between first and second joints
    L2 = 109  # mm, distance between second and third joints
    if (x**2 + y**2)**0.5 > L1+L2:
        print("error, (x,y) too high")
        return False, False, False
    # Step 1: Calculate c and d
    c = (L2**2 - (L1**2 + x**2 + y**2)) / (2 * L1)
    d = math.sqrt(x**2 + y**2 - c**2)
    
    # Step 2: Calculate theta1
    theta1 = math.atan2(x * d - y * c, x * c + y * d)
    theta1_deg = math.degrees(theta1)
    #print(x*math.cos(theta1)-y*math.sin(theta1)-c) # theta1 check
    # Step 3: Calculate alpha
    alpha = math.atan2(y - L1 * math.sin(theta1), x + L1 * math.cos(theta1))
    # print(y-L2*math.sin(alpha)-L1*math.sin(theta1) , x-L2*math.cos(alpha)+L1*math.cos(theta1)) # alpha check

    # Step 4: Calculate a_square and cos(theta2)
    a_square = l1**2 + l4**2 + 2 * l1 * l4 * math.cos(alpha + theta1 + gamma)
    cos_theta2 = (d1**2 + a_square - d2**2) / (2 * d1 * math.sqrt(a_square))
    
    # Step 5: Check if cos_theta2 is within the valid range [-1, 1]
    if cos_theta2 < -1 or cos_theta2 > 1:
        theta2_deg = False  # No valid solution
    else:
        # Calculate the possible values for theta2
        theta2 = math.atan2(math.sqrt(1 - cos_theta2**2), cos_theta2)
        theta2_deg = math.degrees(theta2)

    print()
    return int(theta1_deg), int(theta2_deg), int(math.degrees(alpha))

# Example usage:
#theta1_vals, theta2_vals, alpha = IK(-20, 180)
#print(f"Theta1: {theta1_vals}")
#print(f"Theta2: {theta2_vals}")
#print(f"alpha: {alpha}")


import numpy as np
import matplotlib.pyplot as plt

def trajectory_plan(x0, y0, R, steps):
    """
    Function to generate a closed trajectory plan for a robot's leg.
    
    This function computes the x and y coordinates of a half-circle arc followed by a straight line.
    The trajectory is planned such that the arc starts at the topmost point and the line extends to the end of the circle,
    maintaining a condition that no point of the line should exceed the circle's center y-coordinate.
    
    Parameters:
    x0 (int): x-coordinate for the center of the circle (x0 > 0)
    y0 (int): y-coordinate for the center of the circle (y0 > 0)
    R (int): radius of the circle
    steps (int): number of steps to discretize the arc and line
    
    Returns:
    x (list): x-coordinates of the trajectory
    y (list): y-coordinates of the trajectory
    len_arc (int): length (in steps) of the arc part of the trajectory
    """
    
    # Generate theta values for the half-circle (π to -π radians)
    circle_ang = np.linspace(np.pi, -np.pi, steps)

    # Calculate x and y values for the half-circle with the center at (x0, y0)
    x = (x0 + R) + R * np.cos(circle_ang)
    y = y0 - R * np.sin(circle_ang)
    
    # Apply the condition: if y > y0, set y to y0 (to form a straight line)
    y = np.where(y > y0, y0, y)

    # Calculate the number of steps for the arc (half of the total steps)
    len_arc = steps // 2

    return x.tolist(), y.tolist(), len_arc
'''
def traj_use(x_trj, y_trj, R, steps):
    # Trajectory usage
    thetas1 = []
    thetas2 = []
    alphas = []
    step_angles = [[0 for _ in range(8)] for _ in range(steps)]

    for i in range(steps):
        theta1_temp, theta2_temp, alpha_temp = IK(x_trj[i], y_trj[i])
        thetas1.append(theta1_temp)
        thetas2.append(theta2_temp)
        alphas.append(alpha_temp)

    for s in range(steps):
        if s < (steps/2):
            step_angles[s] = theta2servo(thetas1[s] , thetas2[s] , thetas1[int(s+steps/2)] , thetas2[int(s+steps/2)] , thetas1[int(s-1+steps/2)] , thetas2[int(s-1+steps/2)]  , thetas1[s] , thetas2[s])
        
        elif s >= (steps/2):
            step_angles[s] = theta2servo(thetas1[s] , thetas2[s] , thetas1[int(s-steps/2)] , thetas2[int(s-steps/2)] , thetas1[int(s-steps/2)] , thetas2[int(s-steps/2)]  , thetas1[s] , thetas2[s])

        print(s)
        print(step_angles[s])

    return step_angles
'''

def traj_use(x_trj, y_trj, R, steps):
    # Trajectory usage
    thetas1 = []
    thetas2 = []
    alphas = []
    step_angles = [[0 for _ in range(8)] for _ in range(steps)]
    for i in range(steps):
        theta1_temp, theta2_temp, alpha_temp = IK(x_trj[i], y_trj[i])
        thetas1.append(theta1_temp)
        thetas2.append(theta2_temp)
        alphas.append(alpha_temp)

    for s in range(steps):
        if s < (steps/2):
            step_angles[s] = theta2servo(thetas1[s]  , thetas2[s] , thetas1[int(s+steps/2)] , thetas2[int(s+steps/2)] , thetas1[int(s-1+steps/2)] , thetas2[int(s-1+steps/2)] , thetas1[s] , thetas2[s])
        elif s >= (steps/2):
            step_angles[s] = theta2servo(thetas1[s] , thetas2[s] , thetas1[int(s-steps/2)] , thetas2[int(s-steps/2)] , thetas1[int(s-steps/2)] , thetas2[int(s-steps/2)]  , thetas1[s] , thetas2[s])
        print(s)
        print(step_angles[s])
    
    return step_angles

def turn_angles(step_angles,stand):
    n = len(step_angles)
    R_angles = [[0 for _ in range(8)] for _ in range(n)]
    L_angles = [[0 for _ in range(8)] for _ in range(n)]
    for m in range(n):
        if m < n/2:
            R_angles[m] = [ stand[0] , stand[1] , stand[2] , stand[3] , step_angles[int(m+n/2)][4] , step_angles[int(m+n/2)][5] , stand[6] , stand[7] ]
            L_angles[m] = [ step_angles[m][0] , step_angles[m][1] , stand[2] , stand[3] , stand[4] , stand[5] , stand[6] , stand[7] ]
        else:
            R_angles[m] = [ stand[0] , stand[1] , stand[2] , stand[3] , stand[4] , stand[5] , step_angles[int(m-n/2)][6] , step_angles[int(m-n/2)][7] ]
            L_angles[m] = [ stand[0] , stand[1] , step_angles[int(m)][2] , step_angles[int(m)][3] , stand[4] , stand[5] , stand[6] , stand[7] ]
    return R_angles , L_angles
'''

[48, 136, 73, 163, 103, 100, 122, 104]
1
[44, 95, 56, 150, 97, 67, 126, 145]
2
[67, 115, 48, 151, 114, 80, 103, 125]
3
[73, 148, 48, 151, 122, 79, 97, 92]
4
[56, 135, 44, 110, 126, 120, 114, 105]
5
[48, 136, 67, 130, 103, 100, 122, 104]
'''
