function H = genHomoMatrix(x, y, z, Rx, Ry, Rz)
 

    RotX = [1.0, 0.0, 0.0, 0.0; 0.0, cos(Rx), -sin(Rx), 0.0; 0.0, sin(Rx), cos(Rx), 0.0; 0.0, 0.0, 0.0, 1.0];

    RotY = [cos(Ry), 0.0, sin(Ry), 0.0;  0.0, 1.0, 0.0, 0.0; -sin(Ry), 0.0, cos(Ry), 0.0; 0.0, 0.0, 0.0, 1.0];

    RotZ = [cos(Rz), -sin(Rz), 0.0, 0.0; sin(Rz), cos(Rz), 0.0, 0.0; 0.0, 0.0, 1.0, 0.0; 0.0, 0.0, 0.0, 1.0];

    P = [0.0, 0.0, 0.0, x; 0.0, 0.0, 0.0, y; 0.0, 0.0, 0.0, z; 0.0, 0.0, 0.0, 0.0];

    H = RotZ*RotY*RotX + P;
end