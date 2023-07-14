
syms q1 q2 q3 q4 g real

P2x = L0x + r1x;
P2y = -r1z * sin(q1);
P2z = r1z * cos(q1);

P3x = L0x + L1x + r2y*sin(q2);
P3y = - L1z*sin(q1) - r2y*cos(q2);
P3z = r2z + L1z*cos(q1);

P4x = L0x + L1x + L2y*sin(q2) + r3x*cos(q3);
P4y = - r3y - L1z*sin(q1) - L2z*sin(q1) - L2y*cos(q1)*cos(q2);
P4z = L1z*cos(q1) + L2z*cos(q1) - r3x*sin(q3) - L2y*cos(q2)*sin(q1);

P5x = L0x + L1x + r4x + L2y*sin(q2) + L3y*sin(q2) + L3x*cos(q2)*cos(q3);
P5y = L3x*(sin(q1)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) - L1z*sin(q1) - L2z*sin(q1) - L2y*cos(q1)*cos(q2) - L3y*cos(q1)*cos(q2);
P5z = L1z*cos(q1) - L3x*(cos(q1)*sin(q3) - cos(q3)*sin(q1)*sin(q2)) + L2z*cos(q1) - L2y*cos(q2)*sin(q1) - L3y*cos(q2)*sin(q1);

grav = [0; 0; g];

Jv1(1,1) = diff(P2x,q1); Jv1(1,2) = diff(P2x,q2); Jv1(1,3) = diff(P2x,q3); Jv1(1,4) = diff(P2x,q4);
Jv1(2,1) = diff(P2y,q1); Jv1(2,2) = diff(P2y,q2); Jv1(2,3) = diff(P2y,q3); Jv1(2,4) = diff(P2y,q4);
Jv1(3,1) = diff(P2z,q1); Jv1(3,2) = diff(P2z,q2); Jv1(3,3) = diff(P2z,q3); Jv1(3,4) = diff(P2z,q4);

Jv2(1,1) = diff(P3x,q1); Jv2(1,2) = diff(P3x,q2); Jv2(1,3) = diff(P3x,q3); Jv2(1,4) = diff(P3x,q4);
Jv2(2,1) = diff(P3y,q1); Jv2(2,2) = diff(P3y,q2); Jv2(2,3) = diff(P3y,q3); Jv2(2,4) = diff(P3y,q4);
Jv2(3,1) = diff(P3z,q1); Jv2(3,2) = diff(P3z,q2); Jv2(3,3) = diff(P3z,q3); Jv2(3,4) = diff(P3z,q4);

Jv3(1,1) = diff(P4x,q1); Jv3(1,2) = diff(P4x,q2); Jv3(1,3) = diff(P4x,q3); Jv3(1,4) = diff(P4x,q4);
Jv3(2,1) = diff(P4y,q1); Jv3(2,2) = diff(P4y,q2); Jv3(2,3) = diff(P4y,q3); Jv3(2,4) = diff(P4y,q4);
Jv3(3,1) = diff(P4z,q1); Jv3(3,2) = diff(P4z,q2); Jv3(3,3) = diff(P4z,q3); Jv3(3,4) = diff(P4z,q4);

Jv4(1,1) = diff(P5x,q1); Jv4(1,2) = diff(P5x,q2); Jv4(1,3) = diff(P5x,q3); Jv4(1,4) = diff(P5x,q4);
Jv4(2,1) = diff(P5y,q1); Jv4(2,2) = diff(P5y,q2); Jv4(2,3) = diff(P5y,q3); Jv4(2,4) = diff(P5y,q4);
Jv4(3,1) = diff(P5z,q1); Jv4(3,2) = diff(P5z,q2); Jv4(3,3) = diff(P5z,q3); Jv4(3,4) = diff(P5z,q4);
