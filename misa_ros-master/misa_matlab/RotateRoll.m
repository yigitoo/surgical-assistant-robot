function Rx = RotateRoll(a)

Rx(1,1) = cos(a)*0+1; %The variable a must be used in the first element!
Rx(1,2) = 0;
Rx(1,3) = 0;

Rx(2,1) = 0;
Rx(2,2) = cos(a);
Rx(2,3) = -sin(a);

Rx(3,1) = 0;
Rx(3,2) = sin(a);
Rx(3,3) = cos(a);