#Credit for animation goes to: https://create.arduino.cc/projecthub/zjor/inverted-pendulum-on-a-cart-199d6f
#I used the animation code to test my derivations and matlab results

import numpy as np

import matplotlib
matplotlib.use('TKAgg')

import matplotlib.pyplot as pp
import scipy.integrate as integrate
import matplotlib.animation as animation
from matplotlib.patches import Rectangle

from math import pi
from numpy import sin, cos

# physical constants
m = 0.5
M = 1
g = 9.81
l = 1
I = 1/12*M*l**2
#Linearized a
a = I*(M+m)+m*M*l**2
u = 0

# simulation time
dt = 0.05
Tmax = 20
t = np.arange(0.0, Tmax, dt)

# initial conditions
Y = 0		# pendulum angular velocity
th = 0.02	# pendulum angle
x = .0	# cart position
Z = 0	# cart velocity

state = np.array([th, Y, x, Z])

#Linearized System
A=np.matrix([[0, 1, 0, 0], [m*g*l*(M+m)/a, 0, 0, 0], [0, 0, 0, 1], [-m**2*l**2*g/a, 0, 0, 0]])
B=np.matrix([[0], [-m*l/a], [0], [(I+m*l**2)/a]])
C=np.matrix("1 0 0 0; 0 0 1 0")
D=np.matrix("0;0")

#Pole placement feedback matrix
K=np.matrix("-29.0803; -4.7611; -1.0438; -2.1747")


def derivatives(state, t):
	#For Linearized System
	u = -np.matmul(state, K);
	p = (np.matmul(A,state.T) + B*u).T
	n = []

	for i in range(4):
		n.append(p[0,i])
	return n

	#For Non-linearized System
	# a = ((M+m)*(I+m*l**2)-m*m*l*l*cos(state[0])**2)

	# ds = np.zeros_like(state)

	# ds[0] = state[1]
	# ds[1] = m**2*l**2*sin(state[0])*cos(state[0])*state[1]**2/a + m*g*l*sin(state[0])*(M+m)/a - m*l*cos(state[0])/a*u
	# ds[2] = state[3]
	# ds[3] = (m*l*sin(state[0]))*(I+m*l**2)/a*state[1]**2 - m**2*l**2*g*sin(state[0])/a + (I+m*l**2)/a*u

	# return ds


print("Integrating...")

# integrate your ODE using scipy.integrate.

solution = integrate.odeint(derivatives, state, t)

print("Done")

ths = solution[:, 0]
xs = solution[:, 2]

pxs = l * sin(ths) + xs
pys = l * cos(ths)

fig = pp.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-1.3, 1.3), ylim=(-1.2, 1.0))
ax.set_aspect('equal')
ax.grid()

patch = ax.add_patch(Rectangle((0, 0), 0, 0, linewidth=1, edgecolor='k', facecolor='g'))

line, = ax.plot([], [], 'o-', lw=2)
time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

cart_width = 0.3
cart_height = 0.2

def init():
    line.set_data([], [])
    time_text.set_text('')
    patch.set_xy((-cart_width/2, -cart_height/2))
    patch.set_width(cart_width)
    patch.set_height(cart_height)
    return line, time_text, patch


def animate(i):
    thisx = [xs[i], pxs[i]]
    thisy = [0, pys[i]]

    line.set_data(thisx, thisy)
    time_text.set_text(time_template % (i*dt))
    patch.set_x(xs[i] - cart_width/2)
    return line, time_text, patch

ani = animation.FuncAnimation(fig, animate, np.arange(1, len(solution)),
                              interval=25, blit=True, init_func=init)

pp.show()

# Set up formatting for the movie files
print("Writing video...")
Writer = animation.writers['imagemagick']
writer = Writer(fps=25, metadata=dict(artist='Sergey Royz'), bitrate=1800)
ani.save('free-cart.gif', writer=writer)

