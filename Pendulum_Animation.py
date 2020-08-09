from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import expm
import matplotlib.animation as animation
import time
import Rotation_Matrices
import RBD_Euler_Method_Gravity

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d', xlim=(-1.5, 1.5), ylim=(-1.5, 1.5), zlim=(-1.5, 1.5))

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

e_3 = np.array([0, 0, 1])

# applying the Euler's equations to return rotations
rotation_matrix = RBD_Euler_Method_Gravity.euler_method_gravity_rotation_matrix()
k = len(rotation_matrix)


def plot_pendulum(a):  # vector position of the pendulum bob

    plt.plot([a[0], 0], [a[1], 0], [a[2], 0])
    ax.scatter(a[0], a[1], a[2], c='r', marker='o')

    return


def update_pendulum(i):
    plt.cla()  # the previous frame should be cleared

    # keeping the axes dimensions as constant
    ax = fig.add_subplot(111, projection='3d', xlim=(-1.5, 1.5), ylim=(-1.5, 1.5), zlim=(-1.5, 1.5))

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    # position vectors of the pendulum (in the ith frame) in the inertial frame - rotation about e_3
    point_new = list(np.dot(rotation_matrix[i], e_3))
    # returns the graph of the pendulum
    return plot_pendulum(point_new)


# Creating the Animation object
pendulum_ani = animation.FuncAnimation(fig, update_pendulum, frames=k, interval=1, blit=False, repeat=False)

plt.show()
