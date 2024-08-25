# Closed-Loop system (Simple Pendulum + PD Controller)

import math
import numpy as np

def swingUp_Pendulum(x1,x2,m,l,b,K,u_max,g=9.81):
    # Energy based controller
    E_des = 2*m*g*l # Desired Energy 
    E_act = 0.5*m*(l**2)*(x2**2) + m*g*l*(1 - math.cos(x1))
    # E_act = 0.5*m*(l**2)*(x2**2) - m*g*l*(math.cos(x1))
    if x2*math.cos(x1) >= 0:
        u = K*(E_des - E_act)
    else:
        u = -K*(E_des - E_act)
    # Implementing actuator limits
    u = np.clip(u,-u_max,u_max)
    # State-Space
    x1_dot = x2
    x2_dot = (u/(m*(l**2))) - ((b*x2)/(m*l)) - ((g*math.sin(x1))/l)
    return x1_dot, x2_dot, u
