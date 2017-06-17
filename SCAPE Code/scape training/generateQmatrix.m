function trainQ = generateQmatrix()

segname = cell(1,16);
segname{1} = 'lefthand';segname{2} = 'lowerarm';segname{3} = 'uparm';segname{4} = 'righthand';segname{5} = 'rightlowarm';segname{6} = 'rightuparm';segname{7} = 'leftfoot';
segname{8} = 'leftlowleg';segname{9} = 'leftupleg';segname{10}='rightfoot';segname{11} = 'rightlowleg';segname{12} = 'rightupleg';segname{13} = 'head';segname{14} = 'chest';segname{15} = 'stomach';segname{16} = 'hip';

mesh = importdata('D:\matlab_code\scapecode\scape\1.obj');%'E:\SCAPE\template.obj'
v = mesh.data(1:12500,:);%template model 12500

%tri.txt代表所有的面片 因此tri.txt的大小就是面片的大小
tri = importdata('D:\matlab_code\scapecode\bodyseg\partidx\tri.txt');
tri = tri.data;

trinum =  size(tri,1);
trainQ = [];

for i=2:71%1:34%
    mesh = importdata(strcat('D:\matlab_code\scapecode\scape\',num2str(i),'.obj'));
    %mesh = importdata(strcat('E:\SCAPE\registered\s1p',num2str(i),'.obj'));
    vt = mesh.data(1:12500,:);
    
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

    Q = zeros(trinum*3,3);
    % for seg=1:16
    %   Q = Qmatrix_triangle(seg, parttransforms((seg-1)*3+1:seg*3,:), v, vt, Q);
    % end
    Q = newQmatrix_triangle(parttransforms, v, vt, Q);
    %testQmatrix(R, Q);
    trainQ = [trainQ, Q];
    end

