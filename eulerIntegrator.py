# Numerical Integrator using Forward Euler Scheme

import math
import numpy as np

from stateSpace_Pendulum import stateSpace_Pendulum as pend

# Integration Range
a = 0
b = 5

N = 100 # Integration Steps
# Step-Size
h = (b-a)/N

# Initial Conditions
x1_0 = 60*(math.pi/180)
x2_0 = 0

# Integration Counter
int_count = 0
x1 = x1_0
x2 = x2_0

# Vectors for storing the states
x1Vec = np.zeros(N)
x2Vec = np.zeros(N)

while int_count < N:
    # Function Value
    x1_dot, x2_dot = pend(x1,x2,length=1)
    x1 = x1 + h*x1_dot
    x2 = x2 + h*x2_dot
    x1Vec[int_count] = x1
    x2Vec[int_count] = x2
    int_count += 1