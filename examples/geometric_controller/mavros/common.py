import numpy as np


def matrix_hat(v):
    # Sanity checks on M
    m = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    return m


def matrix_hat_inv(m):
    # TODO: Sanity checks if m is skew symmetric
    v = np.array([m[2, 1], m[0, 2], m[1, 0]])
    return v


def quatMultiplication(q, p):
    quat = np.array([p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3],
                     p[0] * q[1] + p[1] * q[0] - p[2] * q[3] + p[3] * q[2],
                     p[0] * q[2] + p[1] * q[3] + p[2] * q[0] - p[3] * q[1],
                     p[0] * q[3] - p[1] * q[2] + p[2] * q[1] + p[3] * q[0]])
    return quat


def quat2RotMatrix(q):
    rotmat = np.array([[q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3],
                        2 * q[1] * q[2] - 2 * q[0] * q[3],
                        2 * q[0] * q[2] + 2 * q[1] * q[3]],
                       [2 * q[0] * q[3] + 2 * q[1] * q[2],
                        q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3],
                        2 * q[2] * q[3] - 2 * q[0] * q[1]],
                       [2 * q[1] * q[3] - 2 * q[0] * q[2],
                        2 * q[0] * q[1] + 2 * q[2] * q[3],
                        q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]]])
    return rotmat


def rot2Quaternion(R):
    tr = np.trace(R)
    quat = np.zeros(4)
    if tr > 0.0:
        S = np.sqrt(tr + 1.0) * 2.0  # S=4*qw
        quat[0] = 0.25 * S
        quat[1] = (R[2, 1] - R[1, 2]) / S
        quat[2] = (R[0, 2] - R[2, 0]) / S
        quat[3] = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt[1.0 + R[0, 0] - R[1, 1] - R[2, 2]] * 2.0  # S=4*qx
        quat[0] = (R[2, 1] - R[1, 2]) / S
        quat[1] = 0.25 * S
        quat[2] = (R[0, 1] + R[1, 0]) / S
        quat[3] = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt[1.0 + R[1, 1] - R[0, 0] - R[2, 2]] * 2.0  # S=4*qy
        quat[0] = (R[0, 2] - R[2, 0]) / S
        quat[1] = (R[0, 1] + R[1, 0]) / S
        quat[2] = 0.25 * S
        quat[3] = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt[1.0 + R[2, 2] - R[0, 0] - R[1, 1]] * 2.0  # S=4*qz
        quat[0] = (R[1, 0] - R[0, 1]) / S
        quat[1] = (R[0, 2] + R[2, 0]) / S
        quat[2] = (R[1, 2] + R[2, 1]) / S
        quat[3] = 0.25 * S
    return quat
