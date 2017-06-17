function a =  regression_twist2Qmatrix( k, trainQ, trainr)
    regress_Q = [];
    regress_R = [];
    for i = 1: 70
           Q = trainQ(3 * (k - 1) + 1: 3 * (k - 1) + 3, 3 * (i - 1) + 1 : 3 * (i - 1) + 3);
           Q = Q';
           Q = Q(:);
           Q = Q';
           regress_Q = [regress_Q; Q];
           
           R = trainr(k, 7 * (i - 1) + 1: 7 * (i - 1) + 7);
           regress_R = [regress_R; R];
    
    a = regress_R \ regress_Q;
    a = a';
end

