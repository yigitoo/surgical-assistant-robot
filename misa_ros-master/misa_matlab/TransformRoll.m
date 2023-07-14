function Ty = TransformRoll(a,P)

Ty(1,1) = cos(a)*0 + 1; %The variable a must be used in the first element!
Ty(1,2) = 0;
Ty(1,3) = 0;
Ty(1,4) = P(1);

Ty(2,1) = 0;
Ty(2,2) = cos(a);
Ty(2,3) = -sin(a);
Ty(2,4) = P(2);

Ty(3,1) = 0;
Ty(3,2) = sin(a);
Ty(3,3) = cos(a);
Ty(3,4) = P(3);

Ty(4,1) = 0;
Ty(4,2) = 0;
Ty(4,3) = 0;
Ty(4,4) = 1;