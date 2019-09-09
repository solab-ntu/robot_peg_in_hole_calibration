function fval = fun(x)
    addpath(genpath('/data/denso_ws/src/denso_robot_ros/calibration_flow/scripts/5_solveXZ/matlab/yamlMatlab')); 
    A_PATH = '/data/denso_ws/src/denso_robot_ros/calibration_flow/scripts/4_cameraPoseEstimation/goal/As.yaml';
    B_PATH = '/data/denso_ws/src/denso_robot_ros/calibration_flow/scripts/3_autoPose/goal/Bs.yaml';
    As = ReadYaml(A_PATH);
    Bs = ReadYaml(B_PATH);
    EX = [0.0,  1.0,  0.0, -0.04;
         -1.0,  0.0,  0.0,  0.0;
          0.0,  0.0,  1.0,  0.04;
          0.0,  0.0,  0.0,  1.0];
%     EZ = [0.5      ,   0.8660254,   0.0,          0.2398686 ;
%          -0.8660254,   0.5      ,  -0.0,          0.06147114;
%          -0.0      ,   0.0      ,   1.0,          0.001     ;
%           0.0      ,   0.0      ,   0.0,          1.0       ];
    EZ = [0.0      ,   1.0      ,   0.0,          0.255;
         -1.0      ,   0.0      ,   0.0,          0.055;
          0.0      ,   0.0      ,   1.0,          0.002;
          0.0      ,   0.0      ,   0.0,          1.0  ];
      
    n = length(As);
    fval = zeros(1, n);
    
    HX = genHomoMatrix(x(1), x(2), x(3), x(4), x(5), x(6));
    HZ = genHomoMatrix(x(7), x(8), x(9), x(10), x(11), x(12));
    
    for i=1:n
        Ai = cell2mat(As{i});
        Bi = cell2mat(Bs{i});
        residual = HZ - Bi*HX*Ai;
        fval(i) = norm(residual);
    end
    
    ret = norm(fval);


end