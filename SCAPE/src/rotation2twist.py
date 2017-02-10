#The function that translates rotation matrix into twist, from Ma Yi An invitation to 3D Vision

import numpy as np
import math

'''
Input: 3 * 3 Rotation Matrix

Output: 3 * 1 Twist vector
'''
def rotation2twist(matrix_m):
    mid_theta = (np.trace(matrix_m) - 1) / 2
    theta = math.acos(mid_theta)
    t = np.array([[matrix_m[2][1] - matrix_m[1][2]], [matrix_m[0][2] - matrix_m[2][0]], [matrix_m[1][0] - matrix_m[0][1]]]).T
    t = (abs(theta) / (2 * math.sin(theta))) * t
    return t

def test_twist():
    matrix_m = np.array([[1, 0, 0], [0, math.sqrt(2) / 2, -math.sqrt(2) / 2], [0, math.sqrt(2) / 2, math.sqrt(2) / 2]])
    t = rotation2twist(matrix_m)
    print t

if __name__ == '__main__':
    test_twist()