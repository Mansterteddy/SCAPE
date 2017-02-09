#Test procrusters function.

import procrustes
import numpy as np

X = [[0, 1], [0, 0], [1, 0]]
Y = [[-1, 0], [0, 0], [0, 1]]

X = np.array(X)
Y = np.array(Y)

d, z, t = procrustes.procrustes(X, Y, False, False)
print "d: ", d
print "z: ", z
print "t: ", t