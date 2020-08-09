import matplotlib.pyplot as plt
import numpy as np


def make_a_plot(vec):
    print("Initial Time")
    t_initial = float(input())  # initial time
    print("Final Time")
    t_final = float(input())  # final time
    print("Number of iterations")
    number_of_iterations = int(input())  # number of iterations
    del_t = (t_final - t_initial) / number_of_iterations  # sampling time
    t = np.arange(t_initial, t_final, del_t)
    vec_x = []  # x component of a vec
    vec_y = []  # y component of a vec
    vec_z = []  # z component of a vec
    for k in range(0, number_of_iterations):
        vec_x.append(vec[k][0])
        vec_y.append(vec[k][1])
        vec_z.append(vec[k][2])
    fig, ax = plt.subplots()
    # plt.plot(t, vec_x, 'r', label = "X", t, vec_y, 'b', t, vec_z, 'g')
    # plotting dashed lines (all three components together)
    ax.plot(t, vec_x, 'r', label='X')
    ax.plot(t, vec_y, 'b', label='Y')
    ax.plot(t, vec_z, 'g', label='Z')
    ax.legend()
    plt.xlabel('Time')
    plt.ylabel('Angular Velocity')
    plt.show()


# vect = [[1, 2, 3], [2, 3, 4], [4, 5, 6]]
# make_a_plot(vect)
