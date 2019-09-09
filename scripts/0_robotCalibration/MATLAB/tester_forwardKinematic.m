clc;clear;

robot_geometry_parameters = vs6242_descriptor();
%% Test data
% current_joint_angle = [-0.41262005968566484, 0.6997535495965108, 1.4333837187627836, -0.31243836796757574, 1.7966748866739728, 3.1500000001842485]
current_joint_angle = [-0.22056033932875074, 9.379185816715818e-05, 1.5707860471683812, -4.649388227928597e-05, 1.570787073095854, -7.846115094611861e-05]

%% Forward kinematic
[B] = forwardKinematic(current_joint_angle, robot_geometry_parameters)
q = rotm2quat(B(1:3,1:3)); % Convert to quaternion form

%% Inverse kinematic
% x0 = current_joint_angle*rand(1);
x0 = ones(1, 6);

[all_joint_angle] = inverseKinematic(B, x0, robot_geometry_parameters)
[B2] = forwardKinematic(all_joint_angle, robot_geometry_parameters)