from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import expm
import matplotlib.animation as animation
import time
import Rotation_Matrices
import RBD_Euler_Method_Pendulum_Controller
import Plotting

omega = RBD_Euler_Method_Pendulum_Controller.euler_method_pendulum_controller_omega_vector()
Plotting.make_a_plot(omega)
