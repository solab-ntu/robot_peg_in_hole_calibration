function [B] = forwardKinematic(all_joint_angle, robot_geometry_parameters)
%% Compute homogenous matrix between joint frames
%{ 
    * t_i is the motor radians angle of joint {i}
    * H_i is the coordinate transformation matrix between joint {i} and {i-1}
    frames
%}


% Decompose array to individual variables
pCell = num2cell(all_joint_angle);
[t1, t2, t3, t4, t5, t6] = pCell{:};
rg = robot_geometry_parameters;

H_Base = eye(4);
H_1 = homo_mat(t1, rg.j1.rot_axis ,rg.j1.trans);
H_2 = homo_mat(t2, rg.j2.rot_axis ,rg.j2.trans);
H_3 = homo_mat(t3, rg.j3.rot_axis ,rg.j3.trans);
H_4 = homo_mat(t4, rg.j4.rot_axis ,rg.j4.trans);
H_5 = homo_mat(t5, rg.j5.rot_axis ,rg.j5.trans);
H_6 = homo_mat(t6, rg.j6.rot_axis ,rg.j6.trans);

B = H_Base*H_1*H_2*H_3*H_4*H_5*H_6;
end


%% Construct homogenous matrix
function [H] = homo_mat(rot_angle, rot_axis, trans)
    switch rot_axis
        case 'x'
            rot = rot_x(rot_angle);
        case 'y'
            rot = rot_y(rot_angle);
        case 'z'
            rot = rot_z(rot_angle);
        otherwise
            print('Non-supported axis type')
    end

    H = [rot(1:4, 1:3), trans(1:4, 1)];
end

%% Define rotation matrix
function [H] = rot_x(t_x)
    H  = [1.0, 0.0, 0.0, 0.0;
          0.0, cos(t_x), -sin(t_x), 0.0;
          0.0, sin(t_x), cos(t_x), 0.0;
          0.0, 0.0, 0.0, 1.0;];
end

function [H] = rot_y(t_y)
    H  = [cos(t_y), 0.0, sin(t_y), 0.0;
          0.0, 1.0, 0.0, 0.0;
          -sin(t_y), 0.0, cos(t_y), 0.0;
          0.0, 0.0, 0.0, 1.0;];
end

function [H] = rot_z(t_z)
    H  = [cos(t_z), -sin(t_z), 0.0, 0.0;
          sin(t_z), cos(t_z), 0.0, 0.0;
          0.0, 0.0, 1.0, 0.0;
          0.0, 0.0, 0.0, 1.0;];
end