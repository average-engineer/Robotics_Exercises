# Numerical Integrator using Forward Euler Scheme

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from stateSpace_Pendulum import stateSpace_Pendulum as pend
from PD_Pendulum import PD_Pendulum as pend_pd
from swingUp_Pendulum import swingUp_Pendulum as pend_swing

# Switches
visual = False # Visualization
closedLoop = True # Switch between open-loop and closed-loop
controller = 1 # 0: PD, 1: Energy Swing-up

# Integration Range
a = 0
b = 5

N = 1000 # Integration Steps
# Step-Size
h = (b-a)/N
# Time array
t = np.arange(a, b, h)

# Initial Conditions
x1_0 = 30*(math.pi/180)
x2_0 = 0

L = 0.5 # Pendulum length
m = 0.5 # Mass
c = 0.1 # Damping

# Integration Counter
int_count = 1
x1 = x1_0
x2 = x2_0

# Vectors for storing the states
x1Vec = np.zeros(N)
x1Vec[0] = x1_0
x2Vec = np.zeros(N)
x2Vec[0] = x2_0
torque = np.zeros(N)


while int_count < N:
    # Function Value
    if closedLoop:
        if controller == 0:
            Kp = 20
            Kd = 1
            x1_dot, x2_dot, u = pend_pd(x1,x2,m,L,c,Kp,Kd)
            
        elif controller == 1:
            K = 10
            u_max = 1
            x1_dot, x2_dot, u = pend_swing(x1,x2,m,L,c,K,u_max)
        
    else:
        x1_dot, x2_dot, u = pend(x1,x2,L)
    
    x1 = x1 + h*x1_dot
    x2 = x2 + h*x2_dot
    x1Vec[int_count] = x1
    x2Vec[int_count] = x2
    torque[int_count] = u
    int_count += 1
    
#-------------------Visualization----------------------#
# Convert theta to x, y coordinates for plotting
if visual:
    x = L * np.sin(x1Vec)
    y = -L * np.cos(x1Vec)

    # Initialize the animation plot. Make the aspect ratio equal so it looks right.
    fig = plt.figure()
    ax = fig.add_subplot(aspect='equal')
    ax.grid(True)
    # The pendulum rod, in its initial position.
    x0 = x[0]
    y0 = y[0]
    line, = ax.plot([0, x0], [0, y0], lw=3, c='k')
    # The pendulum bob: set zorder so that it is drawn over the pendulum rod.
    bob_radius = 0.08
    circle = ax.add_patch(plt.Circle([x0,y0], bob_radius,
                          fc='r', zorder=3))

    # Set the plot limits so that the pendulum has room to swing!
    ax.set_xlim(-L*1.2, L*1.2)
    ax.set_ylim(-L*1.2, L*1.2)

    def animate(i):
        # Update the animation at frame i
        line.set_data([0, x[i]], [0, y[i]])
        circle.set_center((x[i], y[i]))

    nframes = N
    interval = h * 20
    ani = animation.FuncAnimation(fig, animate, frames=nframes, repeat=True,
                                  interval=interval)
    plt.show()
    ani.save('pendulum.gif')

#-------------------Plotting----------------------#
# Create the figure and two subplots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# Plot theta (angle) over time
ax1.plot(t, x1Vec*180/math.pi, 'b-')
ax1.set_title('Pendulum Angle')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Theta (Degrees)')
ax1.grid(True)

# Plot omega (angular velocity) over time
ax2.plot(t, x2Vec*180/math.pi, 'r-')
ax2.set_title('Pendulum Angular Velocity')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Omega (degrees/s)')
ax2.grid(True)

# Plot actuator torque for closed-loop
if closedLoop:
    plt.figure(figsize=(8, 4))
    plt.plot(t, torque)
    plt.title("Actuator Torque")
    plt.xlabel("Time")
    plt.ylabel("Torque (Nm)")
    plt.grid(True)
    plt.show()


# Adjust layout
plt.tight_layout()
plt.show()