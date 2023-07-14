function Ry = RotatePitch(a)

Ry(1,1) = cos(a);
Ry(1,2) = 0;
Ry(1,3) = sin(a);

Ry(2,1) = 0;
Ry(2,2) = 1;
Ry(2,3) = 0;

Ry(3,1) = -sin(a);
Ry(3,2) = 0;
Ry(3,3) = cos(a);

