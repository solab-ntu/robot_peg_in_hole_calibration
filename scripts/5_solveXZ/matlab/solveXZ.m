clc;clear;


% xStar = [-0.04, 0.0, 0.04, 0, 0, deg2rad(-90), 0.2398686, 0.06147114, 0.001, 0.0, 0.0, deg2rad(-60)];
xStar = [-0.04, 0.0, 0.04, 0, 0, deg2rad(-90), 0.255, 0.055, 0.002, 0.0, 0.0, deg2rad(-90.0)];
x1 = [-0.0400   -0.0000    0.0400   -0.0002    0.0000   -1.5708    0.2549    0.0550    0.0020   -0.0002   -0.0001   -1.5707];

% 
rStar = norm(fun(xStar))
r1 = norm(fun(x1))

% 
% HX = genHomoMatrix(xStar(1), xStar(2), xStar(3), xStar(4), xStar(5), xStar(6))
% HZ = genHomoMatrix(xStar(7), xStar(8), xStar(9), xStar(10), xStar(11), xStar(12))
% 
% HX = genHomoMatrix(x1(1), x1(2), x1(3), x1(4), x1(5), x1(6))
% HZ = genHomoMatrix(x1(7), x1(8), x1(9), x1(10), x1(11), x1(12))

x0 = ones(1, 12);
lb = [-1, -1, -1, 0.0, 0.0, 0.0, -1, -1, -1, 0.0, 0.0, 0.0];
ub = [1, 1, 1, 2*pi, 2*pi, 2*pi, 1, 1, 1, 2*pi, 2*pi, 2*pi];
options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter-detailed');
[x,resnorm,residual,exitflag,output] = lsqnonlin(@fun,xStar,[],[], options)
ri = norm(fun(x))