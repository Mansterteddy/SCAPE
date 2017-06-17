function [U, mu, trainS1] = generateSmatrix()

nearestpart = [2,3; 1,3; 2,14; 5,6; 4,6; 5,14; 8,9; 7,9; 8,16; 11,12; 10,12; 11,16; 3,6; 13,15; 14,16; 9,12];

segname = cell(1,16);
segname{1} = 'lefthand';segname{2} = 'lowerarm';segname{3} = 'uparm';segname{4} = 'righthand';segname{5} = 'rightlowarm';segname{6} = 'rightuparm';segname{7} = 'leftfoot';
segname{8} = 'leftlowleg';segname{9} = 'leftupleg';segname{10}='rightfoot';segname{11} = 'rightlowleg';segname{12} = 'rightupleg';segname{13} = 'head';segname{14} = 'chest';segname{15} = 'stomach';segname{16} = 'hip';

%mesh = importdata('E:\SCAPE\dataset\scape\1.obj');%'E:\MPI_bodyDataset\registered\s1p0.obj'
mesh = importdata('D:\matlab_code\scapecode\scape\1.obj');
v = mesh.data(1:12500,:);%template model 6449

%tripart = importdata('E:\SCAPE\bodyseg\partidx\tripart.txt');
tripart = importdata('D:\matlab_code\scapecode\bodyseg\partidx\tripart.txt');
trinum =  size(tripart,1);

posemodel = importdata('D:\matlab_code\scapecode\blendposemodel.txt');
%posemodel = importdata('E:\SCAPE\blendposemodel.txt');%posemodel_training();%training pose model('E:\MPI_bodyDataset\personposemodel.txt');%

trainS1 = [];
%fileNames = list_image_files('D:\matlab_code\scapecode\SPRING_MALE\')
%fileNames = list_image_files('E:\SCAPE\dataset\SPRING_MALE\');

% fileNames = cell(115,1);
% [poseI, subjectJ] = Tenbotrainingfile();
% tcout = 1;
% for i=2:115
%     if subjectJ(i)>0
%         fileNames{tcout} = strcat('s',num2str(i),'p0.obj');
%         tcout = tcout+1;
%     end
% end
    
for i=1:150

    mesh = importdata(strcat('D:\matlab_code\scapecode\SPRING_MALE_2\SPRING (', num2str(i), ').obj'));%''  E:\MPI_bodyDataset\registered\ 
    vt = mesh.data(1:12500,:);

    [D, yt, Transform] = procrustes(v, vt, 'Scaling',false, 'Reflection',false);
    vt = yt;

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

    S = zeros(trinum*3,3);
% for seg=1:16
%   S = Smatrix_triangle(seg, parttransforms((seg-1)*3+1:seg*3,:), meshr, posemodel, v, vt, S);
% end
    S = newSmatrix_triangle(parttransforms, meshr, posemodel, v, vt, S);
%testSmatrix(parttransforms, S, meshr, posemodel);
    S = reshape(S',[trinum*3*3,1]);
    trainS1 = [trainS1; S'];
end

options.PCARatio = 0.9;
[U, eigenvalue] = PCA(trainS1,options);
%[U, eigenvalue] = PCA(trainS1);
mu = mean(trainS1);