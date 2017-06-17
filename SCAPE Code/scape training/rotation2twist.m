function t =  rotation2twist(matrix_m)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    mid_theta = (trace(matrix_m) - 1) / 2;
    theta = acos(mid_theta);
    t = [matrix_m(3, 2) - matrix_m(2, 3); matrix_m(1, 3) - matrix_m(3, 1); matrix_m(2, 1) - matrix_m(1, 2)]';
    t = (abs(theta) / (2 * sin(theta))) * t;
end

