import numpy as np
import math

# these are anti-clockwise rotation matrices


def rotation_matrix_x(x, y):  # rotation about x axis in 3D, x = angle in degrees, y = vector to be rotated (list)

    theta = np.radians(x)  # degrees input, converted to radians

    r_x = np.array(((1, 0, 0),  # the matrix
                    (0, np.cos(theta), -np.sin(theta)),
                    (0, np.sin(theta), np.cos(theta))))

    # print('rotation matrix along x:')
    # print(r_x)

    v = np.array(y)  # converting the list y into an vector

    # print('vector v: ')
    # print(v)

    # print('apply the rotation matrix r to v: r*v')
    return r_x.dot(v)  # taking the dot product of the desired array with the rotation matrix


def rotation_matrix_y(x, y):  # similarly about y axis in 3D

    theta = np.radians(x)

    r_y = np.array(((np.cos(theta), 0, np.sin(theta)),
                    (0, 1, 0),
                    (-np.sin(theta), 0, np.cos(theta))))

    # print('rotation matrix along y:')
    # print(r_y)

    v = np.array(y)

    # print('vector v: ')
    # print(v)

    # print('apply the rotation matrix r to v: r*v')
    return r_y.dot(v)


def rotation_matrix_z(x, y):  # similarly about z axis in 3D

    theta = np.radians(x)

    r_z = np.array(((np.cos(theta), -np.sin(theta), 0),
                    (np.sin(theta), np.cos(theta), 0),
                    (0, 0, 1)))

    # print('rotation matrix along y:')
    # print(r_z)

    v = np.array(y)

    # print('vector v: ')
    # print(v)

    # print('apply the rotation matrix r to v: r*v')
    return r_z.dot(v)


def euler_to_rotation_matrix(phi):  # Rotation Matrix given euler angles - in degrees (a vector with 3 angles XYZ)
    theta = []
    for i in range(0, 3):
        temp = np.radians(phi[i])
        theta.append(temp)

    r_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]
                    ])

    r_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]
                    ])

    r_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]
                    ])

    r = np.dot(r_z, np.dot(r_y, r_x))

    return r


# Checks if a matrix is a valid rotation matrix.
def is_rotation_matrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).

def rotation_matrix_to_euler(R):  # XYZ Euler angles are returned
    assert (is_rotation_matrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([np.degrees(x), np.degrees(y), np.degrees(z)])
