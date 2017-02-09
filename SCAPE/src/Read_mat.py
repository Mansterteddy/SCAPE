# -*- coding:utf8 -*-
import procrustes
import handle
import load_mat
import numpy as np
import math
import write_obj
import rotation2twist
from scipy import sparse
from scipy.sparse.linalg import *

def parttransform(i):
    mesh_1 = handle.handle_obj("D:/matlab_code/scapecode/scape/1.obj")
    mesh_2 = handle.handle_obj("D:/matlab_code/scapecode/scape/20.obj")

    part_vertex_id = handle.handle_txt(i)

    mesh_1_part = mesh_1[part_vertex_id]
    mesh_2_part = mesh_2[part_vertex_id]

    d, z, t = procrustes.procrustes(mesh_1_part, mesh_2_part, False, False)

    return t['rotation']

def scape_trans():
    nearestpart = [[2, 3], [1, 3], [2, 14], [5, 6], [4, 6], [5, 14], [8, 9], [7, 9], [8, 16]
                   , [11, 12], [10, 12], [11, 16], [3, 6], [13, 15], [14, 16], [9, 12]]

    mat_contents = load_mat.load_mat()
    print "Import Matrix Done"

    tri = handle.handle_tri("D:/matlab_code/scapecode/bodyseg/partidx/tri.txt")
    trinum = len(tri)
    print "trinum: ", trinum

    tripart = handle.handle_tripart("D:/matlab_code/scapecode/bodyseg/partidx/tripart.txt")

    parttransform_mat = []

    for i in xrange(16):
        parttransform_mat_son = parttransform(i+1)
        parttransform_mat.append(parttransform_mat_son)

    mesh_r = []
    for i in xrange(trinum):
        part = tripart[i]
        nearpart = nearestpart[part - 1]
        R = parttransform_mat[part - 1]
        R1 = parttransform_mat[nearpart[0] - 1]
        R2 = parttransform_mat[nearpart[1] - 1]
        relativeR1 = np.dot(R1.T, R)
        relativeR2 = np.dot(R2.T, R)
        twist1 = rotation2twist.rotation2twist(relativeR1)
        twist2 = rotation2twist.rotation2twist(relativeR2)
        mesh_r_son = [twist1[0][0], twist1[0][1], twist1[0][2], twist2[0][0], twist2[0][1], twist2[0][2], 1]
        mesh_r.append(mesh_r_son)

    mesh_r = np.array(mesh_r)

    pose_model = mat_contents['pose_model']
    mesh_Q = []
    for i in xrange(trinum):
        mesh_r_sub = mesh_r[i] # 1 * 7
        pose_model_sub = pose_model[i * 9: (i + 1) * 9]
        mesh_Q_sub = np.dot(pose_model_sub, mesh_r_sub.T)
        mesh_Q_sub = np.resize(mesh_Q_sub, (3, 3))
        mesh_Q.append(mesh_Q_sub)

    eigen_value = mat_contents['eigen_value']
    U = mat_contents['U']
    amu = mat_contents['amu']
    eigen_value_mat = np.zeros((66, 1))
    eigen_value_mat[0][0] = math.sqrt(eigen_value[0][0]) * (0.2)
    mesh_S = np.dot(U, eigen_value_mat) + amu.T
    mesh_S = np.resize(mesh_S, (3 * trinum, 3))

    vertex_axis = []
    whole_vertex = handle.handle_whole("D:/matlab_code/scapecode/bodyseg/whole.txt")
    vertex_num = len(whole_vertex)
    for i in xrange(trinum):
        tri_sub_1 = tri[i][0]
        tri_sub_2 = tri[i][1]
        tri_sub_3 = tri[i][2]

        whole_vertex_1 = whole_vertex[tri_sub_1 - 1]
        whole_vertex_2 = whole_vertex[tri_sub_2 - 1]
        whole_vertex_3 = whole_vertex[tri_sub_3 - 1]

        vertex_axis_son = []
        vertex_axis_son.append(whole_vertex_1)
        vertex_axis_son.append(whole_vertex_2)
        vertex_axis_son.append(whole_vertex_3)

        vertex_axis.append(vertex_axis_son)


    final_b = []
    for i in xrange(trinum):
        mesh_Q_sub = mesh_Q[i]
        mesh_S_sub = mesh_S[3*i: 3*i+3]
        part = tripart[i]
        r_sub = parttransform_mat[part - 1]

        vertex_axis_sub_1 = vertex_axis[i][0]
        vertex_axis_sub_2 = vertex_axis[i][1]
        vertex_axis_sub_3 = vertex_axis[i][2]

        b_final_sub_1 = vertex_axis_sub_2 - vertex_axis_sub_1
        b_final_sub_2 = vertex_axis_sub_3 - vertex_axis_sub_1

        b_final_sub_1 = np.dot(r_sub, np.dot(mesh_S_sub, np.dot(mesh_Q_sub, b_final_sub_1.T)))
        b_final_sub_2 = np.dot(r_sub, np.dot(mesh_S_sub, np.dot(mesh_Q_sub, b_final_sub_2.T)))

        final_b.append(b_final_sub_1.T[0])
        final_b.append(b_final_sub_1.T[1])
        final_b.append(b_final_sub_1.T[2])
        final_b.append(b_final_sub_2.T[0])
        final_b.append(b_final_sub_2.T[1])
        final_b.append(b_final_sub_2.T[2])
        #final_b.append(b_final_sub_1.T)
        #final_b.append(b_final_sub_2.T)

    final_b = np.array(final_b)
    print final_b

    final_A = sparse.csr_matrix((trinum * 2 * 3, vertex_num * 3))
    #final_A = np.zeros((trinum * 2 * 3, vertex_num * 3))
    for i in xrange(trinum):
        tri_sub_1 = tri[i][0]
        tri_sub_2 = tri[i][1]
        tri_sub_3 = tri[i][2]

        final_A[i * 6, (tri_sub_1 - 1) * 3] = -1
        final_A[i * 6, (tri_sub_2 - 1) * 3] = 1
        final_A[i * 6 + 1, (tri_sub_1 - 1) * 3 + 1] = -1
        final_A[i * 6 + 1, (tri_sub_2 - 1) * 3 + 1] = 1
        final_A[i * 6 + 2, (tri_sub_1 - 1) * 3 + 2] = -1
        final_A[i * 6 + 2, (tri_sub_2 - 1) * 3 + 2] = 1
        #final_A[i * 2][tri_sub_1 - 1] = -1
        #final_A[i * 2][tri_sub_2 - 1] = 1

        final_A[i * 6 + 3, (tri_sub_1 - 1) * 3] = -1
        final_A[i * 6 + 3, (tri_sub_3 - 1) * 3] = 1
        final_A[i * 6 + 4, (tri_sub_1 - 1) * 3 + 1] = -1
        final_A[i * 6 + 4, (tri_sub_3 - 1) * 3 + 1] = 1
        final_A[i * 6 + 5, (tri_sub_1 - 1) * 3 + 2] = -1
        final_A[i * 6 + 5, (tri_sub_3 - 1) * 3 + 2] = 1
        #final_A[i * 2 + 1][tri_sub_1 - 1] = -1
        #final_A[i * 2 + 1][tri_sub_3 - 1] = 1

    final_A_2 = final_A[:, 3:]
    #final_A_2 = sparse.csr_matrix(final_A_1)

    final_point = lsqr(final_A_2, final_b)
    #final_point = linalg.lstsq(final_A_1, final_b)[0]

    #write_obj.write_obj(final_point, tri)
    #print "final point: ", final_point
    np.savetxt('../res/res.txt', final_point[0], delimiter = '\n')

    #print "len: ", len(final_point)
    #print "final_point: ", final_point
    #numpy.zeros can not generate sparse matrix, so it run slow
    #从python引入包，不会自动引入所有的包，需要显式地引入要用的包
    #pycharm 中文乱码 要加# -*- coding:utf8 -*-
if __name__ == '__main__':
    scape_trans()
