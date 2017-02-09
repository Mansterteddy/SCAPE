import scipy.io as sio
import types
import numpy as np

def load_mat():
    mat_contents = sio.loadmat("D:/matlab_code/scapecode/trainS1.mat")
    return mat_contents

def load_mat_pose_mode():
    mat_contents = sio.loadmat("D:/matlab_code/scapecode/trainS1.mat")
    pose_model = mat_contents['pose_model']
    pose_model_sub = pose_model[0:9]
    test = np.array([1, 1, 1, 1, 1, 1, 1])
    print test
    res = np.dot(pose_model_sub, test.T)
    print "res: ", res
    res = np.resize(res, (3, 3))
    print "res: ", res

if __name__ == '__main__':
    load_mat_pose_mode()