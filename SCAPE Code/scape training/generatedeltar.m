function trainr = generatedeltar()

nearestpart = [2,3; 1,3; 2,14; 5,6; 4,6; 5,14; 8,9; 7,9; 8,16; 11,12; 10,12; 11,16; 3,6; 13,15; 14,16; 9,12];


%建立各个部位的index
segname = cell(1,16);
segname{1} = 'lefthand';segname{2} = 'lowerarm';segname{3} = 'uparm';segname{4} = 'righthand';segname{5} = 'rightlowarm';segname{6} = 'rightuparm';segname{7} = 'leftfoot';
segname{8} = 'leftlowleg';segname{9} = 'leftupleg';segname{10}='rightfoot';segname{11} = 'rightlowleg';segname{12} = 'rightupleg';segname{13} = 'head';segname{14} = 'chest';segname{15} = 'stomach';segname{16} = 'hip';

%导入标准人体模板
mesh = importdata('D:\matlab_code\scapecode\scape\1.obj');%('E:\MPI_bodyDataset\registered\s1p0.obj');
%v = mesh.data;%template model
v = mesh.data(1:12500,:);

tripart = importdata('D:\matlab_code\scapecode\bodyseg\partidx\tripart.txt');
trinum =  size(tripart,1);

trainr = [];

for i=2:71
    mesh = importdata(strcat('D:\matlab_code\scapecode\scape\',num2str(i),'.obj'));
    %mesh = importdata(strcat('E:\SCAPE\registered\s1p',num2str(i),'.obj'));
    vt = mesh.data(1:12500,:);

    parttransforms = [];
    
    for seg=1:16
        file = strcat('D:\matlab_code\scapecode\bodyseg\partidx\',segname{seg},'.txt');
        partvert = importdata(file);
        partvert = partvert(:,1);

        vpart = v(partvert,:);
        vtpart = vt(partvert,:);
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
    trainr = [trainr, meshr];
end
