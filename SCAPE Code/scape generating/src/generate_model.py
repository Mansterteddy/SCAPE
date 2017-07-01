# -*- coding:utf8 -*-
import procrustes
import handle
import load_mat
import numpy as np
import math
import write_obj
import rotation2twist
import time
import gc
from scipy import sparse
from scipy.sparse.linalg import *

'''
给定两个人体模型，给定某个人体部位，计算两个人体模型之间对应部位的旋转矩阵

t是一个字典，rotation key对应的是旋转矩阵
'''
def parttransform(i):
    #mesh_1 = handle.handle_obj("D:/matlab_code/scapecode/scape/1.obj")
    #mesh_2 = handle.handle_obj("D:/matlab_code/scapecode/scape/20.obj")

    mesh_1 = handle.handle_obj("../res/1.obj")
    mesh_2 = handle.handle_obj("../res/20.obj")

    part_vertex_id = handle.handle_txt(i)

    mesh_1_part = mesh_1[part_vertex_id]
    mesh_2_part = mesh_2[part_vertex_id]

    d, z, t = procrustes.procrustes(mesh_1_part, mesh_2_part, False, False)

    return t['rotation']

'''
核心程序：生成期望pose、shape的人体三维网格模型
'''
def scape_trans():
    #每个部位，对应相邻的两个部位，通过这两个部位，可以得到两个3*1的twist vector，从而可以求出该部位的变形矩阵Q
    nearestpart = [[2, 3], [1, 3], [2, 14], [5, 6], [4, 6], [5, 14], [8, 9], [7, 9], [8, 16]
                   , [11, 12], [10, 12], [11, 16], [3, 6], [13, 15], [14, 16], [9, 12]]

    #载入matlab训练好的矩阵
    mat_contents = load_mat.load_mat()

    pose_model = mat_contents['pose_model']
    eigen_value = mat_contents['eigen_value']
    U = mat_contents['U']
    amu = mat_contents['amu']

    del mat_contents
    gc.collect()

    print "Import Matrix Done"
    print "Import Matrix time", time.clock()

    #tri = handle.handle_tri("D:/matlab_code/scapecode/bodyseg/partidx/tri.txt")
    tri = handle.handle_tri("../../scape training/bodyseg/partidx/tri.txt")
    trinum = len(tri)
    print "trinum: ", trinum

    #tripart = handle.handle_tripart("D:/matlab_code/scapecode/bodyseg/partidx/tripart.txt")
    tripart = handle.handle_tripart("../../scape training/bodyseg/partidx/tripart.txt")
    print "Handle File time", time.clock()

    #保存各个部位的旋转矩阵
    parttransform_mat = []

    for i in xrange(16):
        parttransform_mat_son = parttransform(i+1)
        parttransform_mat.append(parttransform_mat_son)

    print "Part Transform time", time.clock()

    #生成相对旋转twist vector vector是1*7的特征向量
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

    print "Mesh R time", time.clock()

    #根据mesh_r矩阵，以及已经训练好的pose_model矩阵，生成对应的Q矩阵
    mesh_Q = []
    for i in xrange(trinum):
        mesh_r_sub = mesh_r[i] # 1 * 7
        pose_model_sub = pose_model[i * 9: (i + 1) * 9]
        mesh_Q_sub = np.dot(pose_model_sub, mesh_r_sub.T)
        mesh_Q_sub = np.resize(mesh_Q_sub, (3, 3))
        mesh_Q.append(mesh_Q_sub)

    print "Mesh Q time", time.clock()

    #设定参数，生成对应的S矩阵，主要就是调节主对角线元素大小
    eigen_value_mat = np.zeros((72, 1))
    eigen_value_mat[0][0] = math.sqrt(eigen_value[0][0]) * (+0.2)
    mesh_S = np.dot(U, eigen_value_mat) + amu.T
    mesh_S = np.resize(mesh_S, (3 * trinum, 3))

    print "Mesh S time", time.clock()

    #vertex_axis中保存的是待变形三维模型中每一个面上的每一个顶点的三维坐标
    vertex_axis = []
    #whole_vertex = handle.handle_whole("D:/matlab_code/scapecode/bodyseg/whole.txt")
    whole_vertex = handle.handle_whole("../../scape training/bodyseg/whole.txt")
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

    #final_b保存的是最终结果三维模型上每一个三角面片中的两个向量，这两个向量通过source object经过mesh_S, mesh_Q, rotation变形而来
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

    final_b = np.array(final_b)

    print "Final b time", time.clock()

    #生成最终Ax = b的A矩阵
    #矩阵A是个sparse矩阵，生成一个csr_matrix需要三个向量，row、col和data
    indptr = [0]
    indicies = []
    data = []
    for i in xrange(trinum):
        tri_sub_1 = tri[i][0]
        tri_sub_2 = tri[i][1]
        tri_sub_3 = tri[i][2]

        indptr.append((indptr[-1] + 2))
        indicies.append((tri_sub_1 - 1) * 3)
        indicies.append((tri_sub_2 - 1) * 3)
        data.append(-1)
        data.append(1)

        indptr.append((indptr[-1] + 2))
        indicies.append((tri_sub_1 - 1) * 3 + 1)
        indicies.append((tri_sub_2 - 1) * 3 + 1)
        data.append(-1)
        data.append(1)

        indptr.append((indptr[-1] + 2))
        indicies.append((tri_sub_1 - 1) * 3 + 2)
        indicies.append((tri_sub_2 - 1) * 3 + 2)
        data.append(-1)
        data.append(1)

        indptr.append((indptr[-1] + 2))
        indicies.append((tri_sub_1 - 1) * 3)
        indicies.append((tri_sub_3 - 1) * 3)
        data.append(-1)
        data.append(1)

        indptr.append((indptr[-1] + 2))
        indicies.append((tri_sub_1 - 1) * 3 + 1)
        indicies.append((tri_sub_3 - 1) * 3 + 1)
        data.append(-1)
        data.append(1)

        indptr.append((indptr[-1] + 2))
        indicies.append((tri_sub_1 - 1) * 3 + 2)
        indicies.append((tri_sub_3 - 1) * 3 + 2)
        data.append(-1)
        data.append(1)

    data = np.array(data)
    indicies = np.array(indicies)
    indptr = np.array(indptr)

    #生成final_A矩阵
    final_A_1 = sparse.csr_matrix((data, indicies, indptr), shape = (trinum * 2 * 3, vertex_num * 3))
    #为了消除锚点，所以将第一个顶点当做(0, 0, 0)，并从最终的x中删去它
    final_A_2 = final_A_1[:, 3:]

    print "Final A time", time.clock()

    #求解Ax=b中的x，sparse matrix least square求解会更快
    final_point = lsqr(final_A_2, final_b)

    print "Final Point time", time.clock()

    #写入文件
    write_obj.write_obj(final_point[0], tri)

    #如果想导出结果的话，可以采用这种方式
    #np.savetxt('../res/res.txt', final_point[0], delimiter = '\n')
    print "Final time", time.clock()

if __name__ == '__main__':
    scape_trans()
