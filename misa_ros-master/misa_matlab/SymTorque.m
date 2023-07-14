%MISO Robot Forward Kinematics (Symbolic):
clc
clear all
close all

syms l0 l1 h1 q0 q1 q2 q3 q4 g real
syms Le real
syms dq1 dq2 dq3 dq4 ddq1 ddq2 ddq3 ddq4 real
syms I11 I22 I33 I44 real
syms Le g Ld h1 l1 l0 real
syms L0x L1x L1z L2z L2y L3y L3x L4x real
g = 9.806;
%{
Le = 0.156051;
g = 9.807;
Ld = 0.042;
h1 = 0.785;
l1 = 0.185;
L0 = 0.100;
%}
%Transformation Matrix:
%Checked.

Le = 0.156051;
L0x = 0.062;
L1x = Le;
L1z = Le;
L2y = Le;
L2z = Le;
L3x = Le;
L3y = Le;
L4x = 0.042;

T0 = TransformNoRot([0; 0; q0]);
T1 = TransformRoll(q1, [L0x; 0; 0]);
T2 = TransformYaw(q2, [L1x; 0; L1z]);
T3 = TransformPitch(q3, [0; -L2y; L2z]);
T4 = TransformRoll(q4, [L3x; -L3y; 0]);
T5 = TransformNoRot([L4x ; 0 ; 0]);

T01 = T0*T1;
T02 = T01*T2;
T03 = T02*T3;
T04 = T03*T4;
T05 = T04*T5;
%Transformation matrix of the end effector without Endowrist
TEE = simplify(T05);

%matlabFunction(TEE , 'File' , 'FkineTest.m')

%Position of the end effector without endowrist:
xe = TEE(1,4); ye = TEE(2,4); ze = TEE(3,4);

%Mass of link without Endowrist (from Catia V5):
% m1 = M1Rot(0.061kg) + Elbow1(0.121kg) + M2Fix(0.061kg) + Kinova/2(0.1785kg)
% m2 = M2Rot(0.061kg) + Elbow2(0.121kg) + M3Fix(0.061kg) + Kinova(0.357kg)
% m3 = M3Rot(0.061kg) + Elbow3(0.121kg) + M4Fix(0.061kg) + Kinova(0.357kg)
% m4 = M4Rot(0.037kg) + Kinova/2(0.1785kg)

% Kinova - > 380g = 0.38 kg;
% 1 Elb + 2 Pip -> 356 g = 0.356 kg;
% Pip - > 74 g = 0.074 kg;
% m1 = 0.38 + 0.356;
%Note that m0 is not used.

syms m1 m2 m3 m4 payload real
%m1 = 0.73; m2 = 0.73; m3 = 0.73; m4 = 0.235;
m4 = payload + m4;
Grav = [0; 0; g];
%Differentiation Terms for joints:
q = [q1; q2; q3; q4];
dq = [dq1; dq2; dq3; dq4];
ddq = [ddq1; ddq2; ddq3; ddq4];

%Link Inertias (from Catia V5):
%{
I1 = zeros(3);
I1(1,1) = 0.003;
I1(2,2) = 0.001;
I1(3,3) = 0.003;

I2 = zeros(3);
I2(1,1) = 0.002;
I2(2,2) = 0.003;
I2(3,3) = 0.003;

I3 = zeros(3);
I3(1,1) = 0.003;
I3(2,2) = 0.001;
I3(3,3) = 0.003;

I4 = zeros(3);
I4(1,1) = 1.899e-004;
I4(2,2) = 1.852e-004;
I4(3,3) = 2.037e-004;
%}
syms I1 I2 I3 I4;
%Differentiation Terms for joints:
q = [q1; q2; q3; q4];
dq = [dq1; dq2; dq3; dq4];
ddq = [ddq1; ddq2; ddq3; ddq4];

%End points of each link w.r.t the origin:
L01 = T01(1:3,4);
L02 = T02(1:3,4);
L03 = T03(1:3,4);
L04 = T04(1:3,4);
L05 = T05(1:3,4);

%{
CheckPoint
EndPoints = [L01, L02, L03,L04,L05];
matlabFunction(EndPoints, 'File' , 'EndPointsTest');
%}

%Center of mass description of links.
syms r1x r1z r2y r2z r3x r3y r4x real
r1x = 0.09;
r1z = 0.078;
r2z = 0.09;
r2y = 0.078;
r3y = 0.09;
r3x = 0.078;
r4x = 0.02 + payload/(payload + m4) * 0.02;

CR2 = [RotateRoll(q1), [r1x ; 0 ; r1z];
    0 0 0 1];
CR3 = [RotateYaw(q2), [ 0 ; -r2y ; r2z];
    0 0 0 1];
CR4 = [RotatePitch(q3), [r3x ; -r3y ; 0];
    0 0 0 1];
CR5 = [RotateRoll(q4), [r4x ; 0 ; 0];
    0 0 0 1];

P2 = T01 * CR2;
P3 = T02 * CR3;
P4 = T03 * CR4;
P5 = T04 * CR5;

P2 = P2(1:3,4);
P3 = P3(1:3,4);
P4 = P4(1:3,4);
P5 = P5(1:3,4);

R01 = eye(3);
R12 = RotateRoll(q1);
R23 = RotateYaw(q2);
R34 = RotatePitch(q3);
R45 = RotateRoll(q4);

R0 = R01;
R1 = R0*R12;
R2 = R1*R23;
R3 = R2*R34;
R4 = R3*R45;
R0E = simplify(R4);

%CheckPoint
%{
CenterOfMasses = [P2,P3,P4,P5];
matlabFunction(CenterOfMasses, 'File', 'COMTest')
%}

P2x = P2(1); P2y = P2(2); P2z = P2(3);
P3x = P3(1); P3y = P3(2); P3z = P3(3);
P4x = P4(1); P4y = P4(2); P4z = P4(3);
P5x = P5(1); P5y = P5(2); P5z = P5(3);

%Potential Energy of the Links:
V1 = m1 * transpose(P2) * Grav;
V2 = m2 * transpose(P3) * Grav;
V3 = m3 * transpose(P4) * Grav;
V4 = m4 * transpose(P5) * Grav;

%Total Potential Energy:
V = V1 + V2 + V3 + V4;

%Gravity Term for Gravity Compensation:
Grav1 = diff(V,q1);
Grav2 = diff(V,q2);
Grav3 = diff(V,q3);
Grav4 = diff(V,q4);

for i = 1:3
    for j = 1:3
        dR1(i,j) = diff(R1(i,j),q1)*dq1 + diff(R1(i,j),q2)*dq2 + diff(R1(i,j),q3)*dq3 + diff(R1(i,j),q4)*dq4;
        dR2(i,j) = diff(R2(i,j),q1)*dq1 + diff(R2(i,j),q2)*dq2 + diff(R2(i,j),q3)*dq3 + diff(R2(i,j),q4)*dq4;
        dR3(i,j) = diff(R3(i,j),q1)*dq1 + diff(R3(i,j),q2)*dq2 + diff(R3(i,j),q3)*dq3 + diff(R3(i,j),q4)*dq4;
        dR4(i,j) = diff(R4(i,j),q1)*dq1 + diff(R4(i,j),q2)*dq2 + diff(R4(i,j),q3)*dq3 + diff(R4(i,j),q4)*dq4;
    end
end

Wt1 = simplify(dR1*(inv(R1)));
W1(1,1) = Wt1(3,2); Wx1 = W1(1,1);
W1(2,1) = Wt1(1,3); Wy1 = W1(2,1);
W1(3,1) = Wt1(2,1); Wz1 = W1(3,1);

Wt2 = simplify(dR2*(inv(R2)));
W2(1,1) = Wt2(3,2); Wx2 = W2(1,1);
W2(2,1) = Wt2(1,3); Wy2 = W2(2,1);
W2(3,1) = Wt2(2,1); Wz2 = W2(3,1);

Wt3 = simplify(dR3*(inv(R3)));
W3(1,1) = Wt3(3,2); Wx3 = W3(1,1);
W3(2,1) = Wt3(1,3); Wy3 = W3(2,1);
W3(3,1) = Wt3(2,1); Wz3 = W3(3,1);

Wt4 = simplify(dR4*(inv(R4)));
W4(1,1) = Wt4(3,2); Wx4 = W4(1,1);
W4(2,1) = Wt4(1,3); Wy4 = W4(2,1);
W4(3,1) = Wt4(2,1); Wz4 = W4(3,1);
%Jacobian Matrix for Translational Part

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

Gr1 = m1 * Jv1' * Grav;
Gr2 = m2 * Jv2' * Grav;
Gr3 = m3 * Jv3' * Grav;
Gr4 = m4 * Jv4' * Grav;

%Jacobian Rotational Part
Jw1(1,1) = diff(Wx1,dq1); Jw1(1,2) = diff(Wx1,dq2); Jw1(1,3) = diff(Wx1,dq3); Jw1(1,4) = diff(Wx1,dq4);
Jw1(2,1) = diff(Wy1,dq1); Jw1(2,2) = diff(Wy1,dq2); Jw1(2,3) = diff(Wy1,dq3); Jw1(2,4) = diff(Wy1,dq4);
Jw1(3,1) = diff(Wz1,dq1); Jw1(3,2) = diff(Wz1,dq2); Jw1(3,3) = diff(Wz1,dq3); Jw1(3,4) = diff(Wz1,dq4);

Jw2(1,1) = diff(Wx2,dq1); Jw2(1,2) = diff(Wx2,dq2); Jw2(1,3) = diff(Wx2,dq3); Jw2(1,4) = diff(Wx2,dq4);
Jw2(2,1) = diff(Wy2,dq1); Jw2(2,2) = diff(Wy2,dq2); Jw2(2,3) = diff(Wy2,dq3); Jw2(2,4) = diff(Wy2,dq4);
Jw2(3,1) = diff(Wz2,dq1); Jw2(3,2) = diff(Wz2,dq2); Jw2(3,3) = diff(Wz2,dq3); Jw2(3,4) = diff(Wz2,dq4);

Jw3(1,1) = diff(Wx3,dq1); Jw3(1,2) = diff(Wx3,dq2); Jw3(1,3) = diff(Wx3,dq3); Jw3(1,4) = diff(Wx3,dq4);
Jw3(2,1) = diff(Wy3,dq1); Jw3(2,2) = diff(Wy3,dq2); Jw3(2,3) = diff(Wy3,dq3); Jw3(2,4) = diff(Wy3,dq4);
Jw3(3,1) = diff(Wz3,dq1); Jw3(3,2) = diff(Wz3,dq2); Jw3(3,3) = diff(Wz3,dq3); Jw3(3,4) = diff(Wz3,dq4);

Jw4(1,1) = diff(Wx4,dq1); Jw4(1,2) = diff(Wx4,dq2); Jw4(1,3) = diff(Wx4,dq3); Jw4(1,4) = diff(Wx4,dq4);
Jw4(2,1) = diff(Wy4,dq1); Jw4(2,2) = diff(Wy4,dq2); Jw4(2,3) = diff(Wy4,dq3); Jw4(2,4) = diff(Wy4,dq4);
Jw4(3,1) = diff(Wz4,dq1); Jw4(3,2) = diff(Wz4,dq2); Jw4(3,3) = diff(Wz4,dq3); Jw4(3,4) = diff(Wz4,dq4);

M = simplify((m1*transpose(Jv1)*Jv1) + (transpose(Jw1)*transpose(R12)*I1*R12*Jw1)...
    + (m2*transpose(Jv2)*Jv2) + (transpose(Jw2)*transpose(R23)*I2*R23*Jw2) ...
    + (m3*transpose(Jv3)*Jv3) + (transpose(Jw3)*transpose(R34)*I3*R34*Jw3)...
    + (m4*transpose(Jv4)*Jv4) + (transpose(Jw4)*transpose(R45)*I4*R45*Jw4));


dM = simplify(diff(M,q1)*dq1 + diff(M,q2)*dq2 + diff(M,q3)*dq3) + diff(M,q4)*dq4;

Ma = -0.5*dq'*M*dq;

dMa = simplify([diff(Ma,q1); diff(Ma,q2); diff(Ma,q3); diff(Ma,q4)]);

Cc = simplify(dM*dq + M*ddq);

Gr = m1 * Jv1' * Grav + m2 * Jv2' * Grav + m3 * Jv3' *Grav + m4 * Jv4' * Grav;
%
tau = simplify(Cc-dMa+Gr);

matlabFunction(tau, 'File' , 'CalcTorque', 'Optimize', true);

%Gr1, Gr2, Gr3, Gr4

Gr = m1 * Jv1' * Grav + m2 * Jv2' * Grav + m3 * Jv3' *Grav + m4 * Jv4' * Grav;

matlabFunction(Gr, 'File', 'CalcGravity', 'Optimize', true);
