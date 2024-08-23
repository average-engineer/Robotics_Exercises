# State-Space model for a simple pendulum

import math

def stateSpace_Pendulum(x1,x2,length,g = 9.8):
    # x1: Pendulum Angle (1st State)
    # x2: Pendulum Angular rate (2nd State)
    # Length: Length of Pendulum
    # g: Acceleration due to gravity
    x1_dot = x2
    x2_dot = -(g/length)*math.sin(x1)
    
    return x1_dot, x2_dot