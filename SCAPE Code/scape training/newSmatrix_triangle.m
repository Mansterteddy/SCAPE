function S = newSmatrix_triangle(R, meshr, posemodel, v, vt, S)%training, computing the transformation for each triangle in the body part
%partidx: body part index
%R: the rigid part rotation set
%Q: triangle rotation set
%v: vertex of template model
%vt: vertex of target model
%tri: three vertex of each triangle in the template model
%Q: triangle transformation set
segname = cell(1,16);
segname{1} = 'lefthand';segname{2} = 'lowerarm';segname{3} = 'uparm';segname{4} = 'righthand';segname{5} = 'rightlowarm';segname{6} = 'rightuparm';segname{7} = 'leftfoot';
segname{8} = 'leftlowleg';segname{9} = 'leftupleg';segname{10} ='rightfoot';segname{11} = 'rightlowleg';segname{12} = 'rightupleg';segname{13} = 'head';segname{14} = 'chest';segname{15} = 'stomach';segname{16} = 'hip';

tri = importdata('D:\matlab_code\scapecode\bodyseg\partidx\tri.txt');
tri = tri.data;
trinum = size(tri,1);

neighbortriidx = importdata('D:\matlab_code\scapecode\bodyseg\partidx\allneighbortri.txt');%neighbor triangle in the whole body
%neighbortriidx = importdata('E:\SCAPE\bodyseg\partidx\allneighbortri.txt');%neighbor triangle in the whole body
nntrinum = size(neighbortriidx,1);
tripart = importdata('D:\matlab_code\scapecode\bodyseg\partidx\tripart.txt');

totaldim = trinum*2+nntrinum*3;
A = sparse(totaldim,trinum*3); 
b1 = zeros(totaldim,1);
b2 = zeros(totaldim,1);
b3 = zeros(totaldim,1);
for k=1:trinum
    
    vidx = tri(k,:);
    vk2 = v(vidx(2),:) - v(vidx(1),:);
    vk3 = v(vidx(3),:) - v(vidx(1),:);
    vtk2 = vt(vidx(2),:) - vt(vidx(1),:);
    vtk3 = vt(vidx(3),:) - vt(vidx(1),:);  
    
    qk = posemodel(9*(k-1)+1:9*k,:)*meshr(k,:)';%transformation of triangle 
    qk = reshape(qk,[3,3]);
    qkvk2 = qk*vk2';
    qkvk3 = qk*vk3';
    
    A(2*(k-1)+1,3*(k-1)+1) = qkvk2(1);
    A(2*(k-1)+1,3*(k-1)+2) = qkvk2(2);
    A(2*(k-1)+1,3*(k-1)+3) = qkvk2(3);
    A(2*k,3*(k-1)+1) = qkvk3(1);
    A(2*k,3*(k-1)+2) = qkvk3(2);
    A(2*k,3*(k-1)+3) = qkvk3(3);
    
    part = tripart(k);
    Rk = R((part-1)*3+1:part*3,:);
    
    RTvtk2 = Rk'*vtk2';
    RTvtk3 = Rk'*vtk3';
    
    b1(2*(k-1)+1) = RTvtk2(1);
    b1(2*k) = RTvtk3(1);
    b2(2*(k-1)+1) = RTvtk2(2);
    b2(2*k) = RTvtk3(2);
    b3(2*(k-1)+1) = RTvtk2(3);
    b3(2*k) = RTvtk3(3);    
end

ws = 0.03;%0.1;
for k=1:nntrinum
    triidx = neighbortriidx(k,:)-1;%index in the body part
    A(trinum*2+3*(k-1)+1,3*triidx(1)+1) = ws;
    A(trinum*2+3*(k-1)+2,3*triidx(1)+2) = ws;
    A(trinum*2+3*(k-1)+3,3*triidx(1)+3) = ws;
    A(trinum*2+3*(k-1)+1,3*triidx(2)+1) = -ws;
    A(trinum*2+3*(k-1)+2,3*triidx(2)+2) = -ws;
    A(trinum*2+3*(k-1)+3,3*triidx(2)+3) = -ws;    
end

s1 = A\b1;
s2 = A\b2;
s3 = A\b3;
s =[s1';s2';s3'];

for k=1:trinum
    S(3*(k-1)+1:3*k,:) = s(:,3*(k-1)+1:3*k);
end
