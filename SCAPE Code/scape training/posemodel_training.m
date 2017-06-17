function posemodel = posemodel_training()

%bw = importdata('E:\SCAPE\dataset\triskinweight.txt');

    tri = importdata('D:\matlab_code\scapecode\bodyseg\partidx\tri.txt');
    tri = tri.data;
    trinum =  size(tri,1);

    load trainQ;
    load deltar;
%trainQ = generateQmatrix(bw);
%trainr = generatedeltar();

    posemodel = [];
    for k=1:trinum
        %a = regression_twist2Qmatrix(k, trainQ, trainr);%generating regress matrix for each triangle
        a = regression_twist2Qmatrix(k, trainQ, deltar);%generating regress matrix for each triangle
        posemodel = [posemodel;a];
    end

    file = strcat('D:\matlab_code\scapecode\blendposemodel.txt');
    fid = fopen(file,'wt');

    for i=1:size(posemodel,1)
        fprintf(fid,'%d %d %d %d %d %d %d\n',posemodel(i,1),posemodel(i,2),posemodel(i,3),posemodel(i,4),posemodel(i,5),posemodel(i,6),posemodel(i,7));
    end
fclose(fid);