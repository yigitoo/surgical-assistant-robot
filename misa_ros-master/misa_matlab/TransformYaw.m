function Tz = TransformYaw(a, P)

Tz(1,1) = cos(a);
Tz(1,2) = -sin(a);
Tz(1,3) = 0;
Tz(1,4) = P(1);

Tz(2,1) = sin(a);
Tz(2,2) = cos(a);
Tz(2,3) = 0;
Tz(2,4) = P(2);

Tz(3,1) = 0;
Tz(3,2) = 0;
Tz(3,3) = 1;
Tz(3,4) = P(3);

Tz(4,1) = 0;
Tz(4,2) = 0;
Tz(4,3) = 0;
Tz(4,4) = 1;