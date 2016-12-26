function [ parttransforms, meshr, meshQ, vertex_axis, final_A_1 ] = scapetracking_shape_pose()

load pose_model;
load deltar;

nearestpart = [2,3; 1,3; 2,14; 5,6; 4,6; 5,14; 8,9; 7,9; 8,16; 11,12; 10,12; 11,16; 3,6; 13,15; 14,16; 9,12];

segname = cell(1,16);
segname{1} = 'lefthand';segname{2} = 'lowerarm';segname{3} = 'uparm';segname{4} = 'righthand';segname{5} = 'rightlowarm';segname{6} = 'rightuparm';segname{7} = 'leftfoot';
segname{8} = 'leftlowleg';segname{9} = 'leftupleg';segname{10}='rightfoot';segname{11} = 'rightlowleg';segname{12} = 'rightupleg';segname{13} = 'head';segname{14} = 'chest';segname{15} = 'stomach';segname{16} = 'hip';

tri = importdata('D:\matlab_code\scapecode\bodyseg\partidx\tri.txt');
tri = tri.data;
trinum = size(tri, 1);

mesh = importdata('D:\matlab_code\scapecode\scape\1.obj');
v = mesh.data(1: 12500, :);

mesh = importdata(strcat('D:\matlab_code\scapecode\scape\20.obj'));
vt = mesh.data(1:12500, :);

tripart = importdata('D:\matlab_code\scapecode\bodyseg\partidx\tripart.txt');

%每一个部位的旋转矩阵组合成矩阵
parttransforms = [];

for seg=1:16
        file = strcat('D:\matlab_code\scapecode\bodyseg\partidx\',segname{seg},'.txt');
        partvert = importdata(file);
        partvert = partvert(:,1);
    
        %同一部位下 在template obj上的顶点 和 在test obj上的顶点
        vpart = v(partvert,:);
        vtpart = vt(partvert,:);
        %Determine a linear transformation of two datasets 
        [D, Z, Transform] = procrustes(vtpart, vpart, 'Scaling',false, 'Reflection',false);
        parttransforms = [parttransforms; Transform.T'];
end

meshr = [];
for ti=1:trinum
        part = tripart(ti);
        nearpart = nearestpart(part,:);
        R = parttransforms((part-1)*3+1:part*3,:);
        R1 = parttransforms((nearpart(1)-1)*3+1:nearpart(1)*3,:);
        R2 = parttransforms((nearpart(2)-1)*3+1:nearpart(2)*3,:);
        relativeR1 = R1'*R;
        relativeR2 = R2'*R;
        twist1 = rotation2twist(relativeR1);
        twist2 = rotation2twist(relativeR2);
        meshr = [meshr;twist1, twist2, 1];
end

meshQ = [];
for tq=1:trinum
    meshr_sub = meshr(tq, :);
    pose_model_sub = pose_model(1+(tq - 1)*9:9+(tq-1)*9,:);
    meshQ_sub = pose_model_sub * meshr_sub';
    meshQ_sub = (reshape(meshQ_sub', 3, 3))';
    meshQ = [meshQ; meshQ_sub];
end

load eigen_value;
load U;
load amu;
load trainS;

%meshS = trainS(40,:);
%meshS = amu;
%meshS = reshape(meshS, [3, 3*trinum]);
%meshS = meshS';

eigen_value_mat = zeros(28, 1);
eigen_value_mat(1) = sqrt(eigen_value(1)) * (+0.5);
%eigen_value_mat(1) = eigen_value_mat(1) * 11 / 10;
meshS = U * eigen_value_mat + amu';
meshS = reshape(meshS, [3, 3 * trinum]);
meshS = meshS';

whole_vertex = importdata('D:\matlab_code\scapecode\bodyseg\whole.txt');
whole_vertex = whole_vertex.data();
vertex_num = size(whole_vertex, 1);

vertex_axis = [];
for ta = 1:trinum
    tri_sub_1 = tri(ta, 1);
    tri_sub_2 = tri(ta, 2);
    tri_sub_3 = tri(ta, 3);
    whole_vertex_1 = whole_vertex(tri_sub_1,:);
    whole_vertex_2 = whole_vertex(tri_sub_2,:);
    whole_vertex_3 = whole_vertex(tri_sub_3,:);
    vertex_axis = [vertex_axis; whole_vertex_1; whole_vertex_2; whole_vertex_3];
end

final_b = [];
for tb = 1:trinum
    meshQ_sub = meshQ((tb-1)*3+1:(tb-1)*3+3,:);
    meshS_sub = meshS((tb -1)*3+1:(tb-1)*3+3,:);
    part = tripart(tb);
    r_sub = parttransforms((part-1)*3+1:part*3,:);
    
    vertex_axis_sub_1 = vertex_axis((tb-1)*3+1,:);
    vertex_axis_sub_2 = vertex_axis((tb-1)*3+2,:);
    vertex_axis_sub_3 = vertex_axis((tb-1)*3+3,:);
    
    b_final_sub_1 = vertex_axis_sub_2 - vertex_axis_sub_1;
    b_final_sub_2 = vertex_axis_sub_3 - vertex_axis_sub_1;

    b_final_sub_1 = r_sub * meshS_sub * meshQ_sub * b_final_sub_1';
    b_final_sub_2 = r_sub * meshS_sub * meshQ_sub * b_final_sub_2';
    
    final_b = [final_b; b_final_sub_1'; b_final_sub_2'];
end

final_A = sparse(trinum*2, vertex_num);

for taa = 1:trinum
    tri_sub_1 = tri(taa, 1);
    tri_sub_2 = tri(taa, 2);
    tri_sub_3 = tri(taa, 3);
    
    final_A((taa-1)*2+1, tri_sub_1)=-1;
    final_A((taa-1)*2+1, tri_sub_2)=1;

    final_A((taa-1)*2+2, tri_sub_1)=-1;
    final_A((taa-1)*2+2, tri_sub_3)=1;

end

final_A_1 = final_A(:,2:vertex_num);

final_point = final_A_1 \ final_b;

file = strcat('res1.obj');
fid = fopen(file,'wt');
fprintf(fid,'v %d %d %d\n',0,0,0);
for i=1:size(final_point,1)
    fprintf(fid,'v %d %d %d\n',final_point(i,1),final_point(i,2),final_point(i,3));
end
for i=1:size(tri,1)
    fprintf(fid,'f %d %d %d\n',tri(i,1),tri(i,2),tri(i,3));
end
fclose(fid);


end

