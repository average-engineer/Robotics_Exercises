# Closed-Loop system (Simple Pendulum + PD Controller)

import math

def PD_Pendulum(x1,x2,m,l,b,Kp,Kd,g=9.81):
    # PD Controller
    u = Kp*(math.pi - x1) - Kd*x2
    # State-Space
    x1_dot = x2
    x2_dot = (u/(m*(l**2))) - ((b*x2)/(m*l)) - ((g*math.sin(x1))/l)
    return x1_dot, x2_dot, u

