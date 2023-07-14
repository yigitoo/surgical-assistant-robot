% Knee Arthroplasty Surgery Robot Forward Kinematics (Symbolic):

clc
clear all

syms l0 l1 l2 l3 l4 l5 l6 h1 h2 q0 q1 q2 q3 real
syms dq0 dq1 dq2 dq3 dh1 dh2 real
syms ddq0 ddq1 ddq2 ddq3 ddh1 ddh2 real
syms I00 I01 I02 I10 I20 I30 I40 I50 real

% l1 and l5 are the lengths of the primatic joints.
% l6 is the length of the drill bit.
% h1 and h2 are the lengths for showing the amount of the motion of the prismatic joints.
% q0 and q1 are the home position angles for the passive joints.
%Due to the joint configuration problem (no roll rotation), one of the prismatic joints, h1, is fixed.

%Transformation Matrix:
T01 = TransformNoRot([0; 0; (l0+(l1-h1))]);
T12 = TransformYaw(q0, [l2; 0; 0]);
T23 = TransformYaw(q1, ([l3;0;0]));
T34 = TransformYaw(q2, [l4; 0; 0]);
T45 = TransformPitch(q3, [(l5-h2); 0; 0]);
T5E = TransformNoRot([l6; 0; 0]);
T0E = simplify(T01*T12*T23*T34*T45*T5E);

L01 = [1 0 0; 0 1 0; 0 0 1]*[0; 0; (l0+l1-h1)];
L02 = L01+RotateYaw(q0)*[l2; 0; 0];
L03 = L02+RotateYaw(q1)*[l3; 0; 0];
L04 = L03+RotateYaw(q2)*[l4; 0; 0];
L05 = L04+RotatePitch(q3)*[(l5-h2); 0; 0];
L06 = L05+RotatePitch(q3)*[l6; 0; 0];

xe = T0E(1,4);
ye = T0E(2,4);
ze = T0E(3,4);

xedot = diff(xe,q0)*dq0 + diff(xe,q1)*dq1 + diff(xe,q2)*dq2 + diff(xe,q3)*dq3 + diff(xe,h2)*dh2;
yedot = diff(ye,q0)*dq0 + diff(ye,q1)*dq1 + diff(ye,q2)*dq2 + diff(ye,q3)*dq3 + diff(ye,h2)*dh2;
zedot = diff(ze,q0)*dq0 + diff(ze,q1)*dq1 + diff(ze,q2)*dq2 + diff(ze,q3)*dq3 + diff(ze,h2)*dh2;

% Skew symmetric matrix:
R01 = eye(3);
R12 = RotateYaw(q0);
R23 = RotateYaw(q1);
R34 = RotateYaw(q2);
R45 = RotatePitch(q3);
R5E = eye(3); 
R0E = simplify(R01*R12*R23*R34*R45*R5E);

R0 = R01;
R1 = R01*R12;
R2 = R01*R12*R23;
R3 = R01*R12*R23*R34;
R4 = R01*R12*R23*R34*R45;
R5 = R01*R12*R23*R34*R45*R5E;

P0 = [0; 0; 0];
P1 = P0+R01*[0;0;(l0+(l1-h1))/2];
P2 = L01+R12*[l2/2;0;0];
P3 = L02+R23*[l3/2;0;0];
P4 = L03+R34*[l4/2;0;0];
P5 = L04+R45*[(l5-h2)/2;0;0];
P6 = L05+R5E*[l6/2;0;0];

R(1,1) = R0E(1,1); R(1,2) = R0E(1,2); R(1,3) = R0E(1,3);  
R(2,1) = R0E(2,1); R(2,2) = R0E(2,2); R(2,3) = R0E(2,3);
R(3,1) = R0E(3,1); R(3,2) = R0E(3,2); R(3,3) = R0E(3,3);

Rdot = diff(R,q0)*dq0 + diff(R,q1)*dq1 + diff(R,q2)*dq2 + diff(R,q3)*dq3 + diff(R,h2)*dh2;

Wn = Rdot * transpose(R);
W = simplify(Wn);
Wx = W(3,2); Wy = W(1,3); Wz = W(2,1);

% Wxdot = diff(Wx,h1)*dh1 + diff(Wx,q0)*dq0 + diff(Wx,q1)*dq1 + diff(Wx,q2)*dq2 + diff(Wx,q3)*dq3 + diff(Wx,h2)*dh2;
% Wydot = diff(Wy,h1)*dh1 + diff(Wy,q0)*dq0 + diff(Wy,q1)*dq1 + diff(Wy,q2)*dq2 + diff(Wy,q3)*dq3 + diff(Wy,h2)*dh2;
% Wzdot = diff(Wz,h1)*dh1 + diff(Wz,q0)*dq0 + diff(Wz,q1)*dq1 + diff(Wz,q2)*dq2 + diff(Wz,q3)*dq3 + diff(Wz,h2)*dh2;

% Differentiation of Rotation Matrix

for i = 1:3
    for j = 1:3
        dR0(i,j) = diff(R0(i,j),q0)*dq0 + diff(R0(i,j),q1)*dq1 + diff(R0(i,j),q2)*dq2 + diff(R0(i,j),q3)*dq3 + diff(R0(i,j),h2)*dh2;
        dR1(i,j) = diff(R1(i,j),q0)*dq0 + diff(R1(i,j),q1)*dq1 + diff(R1(i,j),q2)*dq2 + diff(R1(i,j),q3)*dq3 + diff(R1(i,j),h2)*dh2;
        dR2(i,j) = diff(R2(i,j),q0)*dq0 + diff(R2(i,j),q1)*dq1 + diff(R2(i,j),q2)*dq2 + diff(R2(i,j),q3)*dq3 + diff(R2(i,j),h2)*dh2;
        dR3(i,j) = diff(R3(i,j),q0)*dq0 + diff(R3(i,j),q1)*dq1 + diff(R3(i,j),q2)*dq2 + diff(R3(i,j),q3)*dq3 + diff(R3(i,j),h2)*dh2;
        dR4(i,j) = diff(R4(i,j),q0)*dq0 + diff(R4(i,j),q1)*dq1 + diff(R4(i,j),q2)*dq2 + diff(R4(i,j),q3)*dq3 + diff(R4(i,j),h2)*dh2;
        dR5(i,j) = diff(R5(i,j),q0)*dq0 + diff(R5(i,j),q1)*dq1 + diff(R5(i,j),q2)*dq2 + diff(R5(i,j),q3)*dq3 + diff(R5(i,j),h2)*dh2;
    end
end


Wt0 = simplify(dR0*(transpose(R0)));
W0(1,1) = Wt0(3,2); Wx0 = W0(1,1);
W0(2,1) = Wt0(1,3); Wy0 = W0(2,1);
W0(3,1) = Wt0(2,1); Wz0 = W0(3,1);


Wt1 = simplify(dR1*(transpose(R1)));
W1(1,1) = Wt1(3,2); Wx1 = W1(1,1);
W1(2,1) = Wt1(1,3); Wy1 = W1(2,1);
W1(3,1) = Wt1(2,1); Wz1 = W1(3,1);

Wt2 = simplify(dR2*(transpose(R2)));
W2(1,1) = Wt2(3,2); Wx2 = W2(1,1);
W2(2,1) = Wt2(1,3); Wy2 = W2(2,1);
W2(3,1) = Wt2(2,1); Wz2 = W2(3,1);

Wt3 = simplify(dR3*(transpose(R3)));
W3(1,1) = Wt3(3,2); Wx3 = W3(1,1);
W3(2,1) = Wt3(1,3); Wy3 = W3(2,1);
W3(3,1) = Wt3(2,1); Wz3 = W3(3,1);

Wt4 = simplify(dR4*(transpose(R4)));
W4(1,1) = Wt4(3,2); Wx4 = W4(1,1);
W4(2,1) = Wt4(1,3); Wy4 = W4(2,1);
W4(3,1) = Wt4(2,1); Wz4 = W4(3,1);

Wt5 = simplify(dR5*(transpose(R5)));
W5(1,1) = Wt5(3,2); Wx5 = W5(1,1);
W5(2,1) = Wt5(1,3); Wy5 = W5(2,1);
W5(3,1) = Wt5(2,1); Wz5 = W5(3,1);


% Inverse Kinematics:
% Jacobian Matrix
% Translation Part:


J(1,1) = diff(xedot,dq0); J(1,2) = diff(xedot,dq1); J(1,3) = diff(xedot,dq2); J(1,4) = diff(xedot,dq3); J(1,5) = diff(xedot,dh2);
J(2,1) = diff(yedot,dq0); J(2,2) = diff(yedot,dq1); J(2,3) = diff(yedot,dq2); J(2,4) = diff(yedot,dq3); J(2,5) = diff(yedot,dh2); 
J(3,1) = diff(zedot,dq0); J(3,2) = diff(zedot,dq1); J(3,3) = diff(zedot,dq2); J(3,4) = diff(zedot,dq3); J(3,5) = diff(zedot,dh2);

% Rotation Part:
 
% J(4,1) = diff(Wx,dq0); J(4,2) = diff(Wx,dq1); J(4,3) = diff(Wx,dq2); J(4,4) = diff(Wx,dq3); J(4,5) = diff(Wx,dh2);
J(4,1) = diff(Wy,dq0); J(4,2) = diff(Wy,dq1); J(4,3) = diff(Wy,dq2); J(4,4) = diff(Wy,dq3); J(4,5) = diff(Wy,dh2);
J(4,1) = diff(Wz,dq0); J(5,2) = diff(Wz,dq1); J(5,3) = diff(Wz,dq2); J(5,4) = diff(Wz,dq3); J(5,5) = diff(Wz,dh2);

J_inv = inv(J);
J_inv = simplify(J_inv);
J = simplify(J);


% Inverse Dynamics:

 syms m0 m1 m2 m3 m4 m5 m6 real
 syms I0 I1 I2 I3 I4 I5 I6 real
 syms g real
 
 Grav = [0; 0; g];
 q = [q0; q1; q2; q3; h2];
 dq = [dq0; dq1; dq2; dq3; dh2];
 ddq = [ddq0; ddq1; ddq2; ddq3; ddh2];
 
I0 = zeros(3,3)*I00;
I0(1,1) = I00;
I0(2,2) = I01;
I0(3,3) = I02;
I1 = zeros(3,3)*I10;
I1(1,1) = 0.012;
I1(2,2) = 0.0713;
I1(3,3) = 0.024;
I2 = zeros(3,3)*I20;
I2(1,1) = 0.0005;
I2(2,2) = 0.0218;
I2(3,3) = 0.0005;
I3 = zeros(3,3)*I30;
I3(1,1) = 0.006;
I3(2,2) = 0.0219;
I3(3,3) = 0.0006;
I4 = zeros(3,3)*I40;
I4(1,1) = 0.00578;
I4(2,2) = 0.003;
I4(3,3) = 0.0008;
I5 = zeros(3,3)*I50;
I5(1,1) = 0.0012;
I5(2,2) = 0.0004;
I5(3,3) = 0.00025;

Jv0(1,1) = diff(P0(1),q0); Jv0(1,2) = diff(P0(1),q1); Jv0(1,3) = diff(P0(1),q2); Jv0(1,4) = diff(P0(1),q3); Jv0(1,5) = diff(P0(1),h2);
Jv0(2,1) = diff(P0(2),q0); Jv0(2,2) = diff(P0(2),q1); Jv0(2,3) = diff(P0(2),q2); Jv0(2,4) = diff(P0(2),q3); Jv0(2,5) = diff(P0(2),h2);
Jv0(3,1) = diff(P0(3),q0); Jv0(3,2) = diff(P0(3),q1); Jv0(3,3) = diff(P0(3),q2); Jv0(3,4) = diff(P0(3),q3); Jv0(3,5) = diff(P0(3),h2);

Jv1(1,1) = diff(P1(1),q0); Jv1(1,2) = diff(P1(1),q1); Jv1(1,3) = diff(P1(1),q2); Jv1(1,4) = diff(P1(1),q3); Jv1(1,5) = diff(P1(1),h2);
Jv1(2,1) = diff(P1(2),q0); Jv1(2,2) = diff(P1(2),q1); Jv1(2,3) = diff(P1(2),q2); Jv1(2,4) = diff(P1(2),q3); Jv1(2,5) = diff(P1(2),h2);
Jv1(3,1) = diff(P1(3),q0); Jv1(3,2) = diff(P1(3),q1); Jv1(3,3) = diff(P1(3),q2); Jv1(3,4) = diff(P1(3),q3); Jv1(3,5) = diff(P1(3),h2);

Jv2(1,1) = diff(P2(1),q0); Jv2(1,2) = diff(P2(1),q1); Jv2(1,3) = diff(P2(1),q2); Jv2(1,4) = diff(P2(1),q3); Jv2(1,5) = diff(P2(1),h2);
Jv2(2,1) = diff(P2(2),q0); Jv2(2,2) = diff(P2(2),q1); Jv2(2,3) = diff(P2(2),q2); Jv2(2,4) = diff(P2(2),q3); Jv2(2,5) = diff(P2(2),h2);
Jv2(3,1) = diff(P2(3),q0); Jv2(3,2) = diff(P2(3),q1); Jv2(3,3) = diff(P2(3),q2); Jv2(3,4) = diff(P2(3),q3); Jv2(3,5) = diff(P2(3),h2);

Jv3(1,1) = diff(P3(1),q0); Jv3(1,2) = diff(P3(1),q1); Jv3(1,3) = diff(P3(1),q2); Jv3(1,4) = diff(P3(1),q3); Jv3(1,5) = diff(P3(1),h2);
Jv3(2,1) = diff(P3(2),q0); Jv3(2,2) = diff(P3(2),q1); Jv3(2,3) = diff(P3(2),q2); Jv3(2,4) = diff(P3(2),q3); Jv3(2,5) = diff(P3(2),h2);
Jv3(3,1) = diff(P3(3),q0); Jv3(3,2) = diff(P3(3),q1); Jv3(3,3) = diff(P3(3),q2); Jv3(3,4) = diff(P3(3),q3); Jv3(3,5) = diff(P3(3),h2);

Jv4(1,1) = diff(P4(1),q0); Jv4(1,2) = diff(P4(1),q1); Jv4(1,3) = diff(P4(1),q2); Jv4(1,4) = diff(P4(1),q3); Jv4(1,5) = diff(P4(1),h2);
Jv4(2,1) = diff(P4(2),q0); Jv4(2,2) = diff(P4(2),q1); Jv4(2,3) = diff(P4(2),q2); Jv4(2,4) = diff(P4(2),q3); Jv4(2,5) = diff(P4(2),h2);
Jv4(3,1) = diff(P4(3),q0); Jv4(3,2) = diff(P4(3),q1); Jv4(3,3) = diff(P4(3),q2); Jv4(3,4) = diff(P4(3),q3); Jv4(3,5) = diff(P4(3),h2);

Jv5(1,1) = diff(P5(1),q0); Jv5(1,2) = diff(P5(1),q1); Jv5(1,3) = diff(P5(1),q2); Jv5(1,4) = diff(P5(1),q3); Jv5(1,5) = diff(P5(1),h2);
Jv5(2,1) = diff(P5(2),q0); Jv5(2,2) = diff(P5(2),q1); Jv5(2,3) = diff(P5(2),q2); Jv5(2,4) = diff(P5(2),q3); Jv5(2,5) = diff(P5(2),h2);
Jv5(3,1) = diff(P5(3),q0); Jv5(3,2) = diff(P5(3),q1); Jv5(3,3) = diff(P5(3),q2); Jv5(3,4) = diff(P5(3),q3); Jv5(3,5) = diff(P5(3),h2);

Jv6(1,1) = diff(P6(1),q0); Jv6(1,2) = diff(P6(1),q1); Jv6(1,3) = diff(P6(1),q2); Jv6(1,4) = diff(P6(1),q3); Jv6(1,5) = diff(P6(1),h2);
Jv6(2,1) = diff(P6(2),q0); Jv6(2,2) = diff(P6(2),q1); Jv6(2,3) = diff(P6(2),q2); Jv6(2,4) = diff(P6(2),q3); Jv6(2,5) = diff(P6(2),h2);
Jv6(3,1) = diff(P6(3),q0); Jv6(3,2) = diff(P6(3),q1); Jv6(3,3) = diff(P6(3),q2); Jv6(3,4) = diff(P6(3),q3); Jv6(3,5) = diff(P6(3),h2);

Jw0(1,1) = diff(Wx0,dq0); Jw0(1,2) = diff(Wx0,dq1); Jw0(1,3) = diff(Wx0,dq2); Jw0(1,4) = diff(Wx0,dq3); Jw0(1,5) = diff(Wx0,dh2);
Jw0(2,1) = diff(Wy0,dq0); Jw0(2,2) = diff(Wy0,dq1); Jw0(2,3) = diff(Wy0,dq2); Jw0(2,4) = diff(Wy0,dq3); Jw0(2,5) = diff(Wy0,dh2);
Jw0(3,1) = diff(Wz0,dq0); Jw0(3,2) = diff(Wz0,dq1); Jw0(3,3) = diff(Wz0,dq2); Jw0(3,4) = diff(Wz0,dq3); Jw0(3,5) = diff(Wz0,dh2);

Jw1(1,1) = diff(Wx1,dq0); Jw1(1,2) = diff(Wx1,dq1); Jw1(1,3) = diff(Wx1,dq2); Jw1(1,4) = diff(Wx1,dq3); Jw1(1,5) = diff(Wx1,dh2);
Jw1(2,1) = diff(Wy1,dq0); Jw1(2,2) = diff(Wy1,dq1); Jw1(2,3) = diff(Wy1,dq2); Jw1(2,4) = diff(Wy1,dq3); Jw1(2,5) = diff(Wy1,dh2);
Jw1(3,1) = diff(Wz1,dq0); Jw1(3,2) = diff(Wz1,dq1); Jw1(3,3) = diff(Wz1,dq2); Jw1(3,4) = diff(Wz1,dq3); Jw1(3,5) = diff(Wz1,dh2);

Jw2(1,1) = diff(Wx2,dq0); Jw2(1,2) = diff(Wx2,dq1); Jw2(1,3) = diff(Wx2,dq2); Jw2(1,4) = diff(Wx2,dq3); Jw2(1,5) = diff(Wx2,dh2);
Jw2(2,1) = diff(Wy2,dq0); Jw2(2,2) = diff(Wy2,dq1); Jw2(2,3) = diff(Wy2,dq2); Jw2(2,4) = diff(Wy2,dq3); Jw2(2,5) = diff(Wy2,dh2);
Jw2(3,1) = diff(Wz2,dq0); Jw2(3,2) = diff(Wz2,dq1); Jw2(3,3) = diff(Wz2,dq2); Jw2(3,4) = diff(Wz2,dq3); Jw2(3,5) = diff(Wz2,dh2);

Jw3(1,1) = diff(Wx3,dq0); Jw3(1,2) = diff(Wx3,dq1); Jw3(1,3) = diff(Wx3,dq2); Jw3(1,4) = diff(Wx3,dq3); Jw3(1,5) = diff(Wx3,dh2);
Jw3(2,1) = diff(Wy3,dq0); Jw3(2,2) = diff(Wy3,dq1); Jw3(2,3) = diff(Wy3,dq2); Jw3(2,4) = diff(Wy3,dq3); Jw3(2,5) = diff(Wy3,dh2);
Jw3(3,1) = diff(Wz3,dq0); Jw3(3,2) = diff(Wz3,dq1); Jw3(3,3) = diff(Wz3,dq2); Jw3(3,4) = diff(Wz3,dq3); Jw3(3,5) = diff(Wz3,dh2);

Jw4(1,1) = diff(Wx4,dq0); Jw4(1,2) = diff(Wx4,dq1); Jw4(1,3) = diff(Wx4,dq2); Jw4(1,4) = diff(Wx4,dq3); Jw4(1,5) = diff(Wx4,dh2);
Jw4(2,1) = diff(Wy4,dq0); Jw4(2,2) = diff(Wy4,dq1); Jw4(2,3) = diff(Wy4,dq2); Jw4(2,4) = diff(Wy4,dq3); Jw4(2,5) = diff(Wy4,dh2);
Jw4(3,1) = diff(Wz4,dq0); Jw4(3,2) = diff(Wz4,dq1); Jw4(3,3) = diff(Wz4,dq2); Jw4(3,4) = diff(Wz4,dq3); Jw4(3,5) = diff(Wz4,dh2);

Jw5(1,1) = diff(Wx5,dq0); Jw5(1,2) = diff(Wx5,dq1); Jw5(1,3) = diff(Wx5,dq2); Jw5(1,4) = diff(Wx5,dq3); Jw5(1,5) = diff(Wx5,dh2);
Jw5(2,1) = diff(Wy5,dq0); Jw5(2,2) = diff(Wy5,dq1); Jw5(2,3) = diff(Wy5,dq2); Jw5(2,4) = diff(Wy5,dq3); Jw5(2,5) = diff(Wy5,dh2);
Jw5(3,1) = diff(Wz5,dq0); Jw5(3,2) = diff(Wz5,dq1); Jw5(3,3) = diff(Wz5,dq2); Jw5(3,4) = diff(Wz5,dq3); Jw5(3,5) = diff(Wz5,dh2);


M = (m0*Jv0'*Jv0)+ Jw0'*R01'*I0*R01*Jw1...
    + (m1*transpose(Jv1)*Jv1) + (transpose(Jw1)*transpose(R12)*I0*R12*Jw1)...
    + (m2*transpose(Jv2)*Jv2) + (transpose(Jw2)*transpose(R23)*I2*R23*Jw2) ...
    + (m3*transpose(Jv3)*Jv3) + (transpose(Jw3)*transpose(R34)*I3*R34*Jw3)...
    + (m4*transpose(Jv4)*Jv4) + (transpose(Jw4)*transpose(R45)*I4*R45*Jw4)...
    + (m5*transpose(Jv5)*Jv5) + (transpose(Jw5)*transpose(R5E)*I5*R5E*Jw5)...
    + (m6*transpose(Jv6)*Jv6);
 
dM = simplify(diff(M,q0)*dq0+diff(M,q1)*dq1+diff(M,q2)*dq2+diff(M,q3)*dq3+diff(M,h2)*dh2);

Ma = -0.5*dq'*M*dq; 
dMa = simplify([diff(Ma,q0); diff(Ma,q1); diff(Ma,q2); diff(Ma,q3); diff(Ma,h2)]);

Cc = simplify(dM*dq + M*ddq);


Gr = m0*Jv0'*Grav+m1*Jv1'*Grav + m2*Jv2'*Grav + m3*Jv3'*Grav + m4*Jv4'*Grav + m5*Jv5'*Grav + m6*Jv6'*Grav;

T = simplify(Cc-dMa+Gr);

% Torques of the Joints:

T1 = T(1);
T2 = T(2);
T3 = T(3);
T4 = T(4);
T5 = T(5);