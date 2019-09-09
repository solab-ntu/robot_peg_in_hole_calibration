function [robot_geometry_parameters] = vs6242_descriptor()
    % Translation displacement between two joints
    robot_geometry_parameters.j1.trans = [0.0; 0.0; 0.0; 1.0];
    robot_geometry_parameters.j2.trans = [0.0; 0.0; 0.28; 1.0];
    robot_geometry_parameters.j3.trans = [0.0; 0.0; 0.21; 1.0];
    robot_geometry_parameters.j4.trans = [-0.075; 0.0; -0.49; 1.0];
    robot_geometry_parameters.j5.trans = [0.0; 0.0; 0.7; 1.0];
    robot_geometry_parameters.j6.trans = [0.0; 0.0; 0.07; 1.0];
    
    % Roation axis of joint
    robot_geometry_parameters.j1.rot_axis = 'z';
    robot_geometry_parameters.j2.rot_axis = 'y';
    robot_geometry_parameters.j3.rot_axis = 'y';
    robot_geometry_parameters.j4.rot_axis = 'z';
    robot_geometry_parameters.j5.rot_axis = 'y';
    robot_geometry_parameters.j6.rot_axis = 'z';
    
    
end