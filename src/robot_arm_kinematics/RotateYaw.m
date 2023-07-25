function Rz = RotateYaw(a)

Rz(1,1) = cos(a);
Rz(1,2) = -sin(a);
Rz(1,3) = 0;

Rz(2,1) = sin(a);
Rz(2,2) = cos(a);
Rz(2,3) = 0;

Rz(3,1) = 0;
Rz(3,2) = 0;
Rz(3,3) = 1;