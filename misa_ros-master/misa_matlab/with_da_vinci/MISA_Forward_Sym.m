%MISO Robot Forward Kinematics (Symbolic):
clc
clear all
close all
% l0 = Length of the base
% l1 = Length the linear module
% h1 = Length of the distance traveled by the linear module
% l2 = x-axis length of the roll joint(after the actuator)
% q1 = Roll joint
% l3 = x-axis length of the elbow_1
% l4 = y-axis length of the elbow_1
% l5 = y-axis length of the M2Fix joint
% q2 = Pitch joint
% l6 = y-axis length of the pitch(M2Rot) joint
% l7 = y-axis length of the elbow_2
% l8 = z-axis length of the elbow_2
% l9 = z-axis length of the yaw(M3Fix) joint
% q3 = Yaw joint
% l10 = z-axis length of the yaw(M3Rot) joint
% l11 = z-axis length of the elbow_3
% l12 = x-axis length of the elbow_3
% l13 = x-axis length of the roll joint_2
% q4 = Roll joint_2
% l14 = x-axis length of the roll joint_2
syms l0 l1 h1 l2 q1 l3 l4 l5 q2 l6 l7 l8 q3 l9 l10 l11 l12 l13 l14 q4 g 
syms dq1 dq2 dq3 dq4 ddq1 ddq2 ddq3 ddq4
syms I11 I22 I33 I44
%Transformation Matrix:
T01 = TransformNoRot([0; 0; l0+(l1-h1)]); %Base to the roll joint including linear module motion h1
T12 = TransformRoll(q1, [(l2+l3); (l4+l5); 0]);
T23 = TransformPitch(q2, [0; (l6+l7); (l8+l9)]);
T34 = TransformYaw(q3, [(l12+l13); 0; (l10+l11)]);
T45 = TransformRoll(q4, [l14; 0; 0]);
%Transformation matrix of the end effector without Endowrist
TEE = simplify(T01*T12*T23*T34*T45);
%Position of the end effector without endowrist:
xe = TEE(1,4); ye = TEE(2,4); ze = TEE(3,4);
%Mass of link without Endowrist (from Catia V5):
% m1 = M1Rot(0.061kg) + Elbow1(0.121kg) + M2Fix(0.061kg) + Kinova/2(0.1785kg)
% m2 = M2Rot(0.061kg) + Elbow2(0.121kg) + M3Fix(0.061kg) + Kinova(0.357kg)
% m3 = M3Rot(0.061kg) + Elbow3(0.121kg) + M4Fix(0.061kg) + Kinova(0.357kg)
% m4 = M4Rot(0.037kg) + Kinova/2(0.1785kg)
m1 = 0.4215; m2 = 0.6000; m3 = 0.6000; m4 = 0.2155;
%Gravity:
Grav = [0; 0; g];
%Differentiation Terms for joints:
q = [q1; q2; q3; q4];
dq = [dq1; dq2; dq3; dq4];
ddq = [ddq1; ddq2; ddq3; ddq4];
%Link Inertias (from Catia V5):
I1 = zeros(3,3)*I11;
I1(1,1) = 0.003;
I1(2,2) = 0.001;
I1(3,3) = 0.003;
I2 = zeros(3,3)*I22;
I2(1,1) = 0.002;
I2(2,2) = 0.003;
I2(3,3) = 0.003;
I3 = zeros(3,3)*I33;
I3(1,1) = 0.003;
I3(2,2) = 0.001;
I3(3,3) = 0.003;
I4 = zeros(3,3)*I44;
I4(1,1) = 1.899e-004;
I4(2,2) = 1.852e-004;
I4(3,3) = 2.037e-004;
%End points of each link w.r.t the origin:
L01 = eye(3)*[0; 0; l0+(l1-h1)];
L02 = L01 + RotateRoll(q1)*[(l2+l3); (l4+l5); 0];
L03 = L02 + RotatePitch(q2)*[0; (l6+l7); (l8+l9)];
L04 = L03 + RotateYaw(q3)*[(l12+l13); 0; (l10+l11)];
L05 = L04 + RotateRoll(q4)*[l14; 0; 0];
%Rotations:
R01 = eye(3);
R12 = RotateRoll(q1);
R23 = RotatePitch(q2);
R34 = RotateYaw(q3);
R45 = RotateRoll(q4);
R0E = simplify(R01*R12*R23*R34*R45);
%Rotations at the end of the joints:
R0 = R01;
R1 = R01*R12;
R2 = R01*R12*R23;
R3 = R01*R12*R23*R34;
R4 = R01*R12*R23*R34*R45;
%For Potential Energy(CoM location w.r.t joint angles):
%P1 for prismatic joint
P0 = [0; 0; 0];
P1 = P0+R01*[0;0;(l0+(l1-h1))/2];
P2 = L01+R12*[(l2+l3)/2; (l4+l5)/2; 0];
P3 = L02+R23*[0; (l6+l7)/2; (l8+l9)/2];
P4 = L03+R34*[(l12+l13)/2; 0; (l10+l11)/2];
P5 = L04+R45*[l14/2; 0; 0];

P2x = P2(1); P2y = P2(2); P2z = P2(3);
P3x = P3(1); P3y = P3(2); P3z = P3(3);
P4x = P4(1); P4y = P4(2); P4z = P4(3);
P5x = P5(1); P5y = P5(2); P5z = P5(3);
%Distance from the origin for the links' CoM(for gravity)
Link_1coo = sqrt(P2(1,1)^2+P2(2,1)^2+P2(3,1)^2);
Link_2coo = sqrt(P3(1,1)^2+P3(2,1)^2+P3(3,1)^2);
Link_3coo = sqrt(P4(1,1)^2+P4(2,1)^2+P4(3,1)^2);
Link_4coo = sqrt(P5(1,1)^2+P5(2,1)^2+P5(3,1)^2);
%Potential Energy of the Links:
V1 = m1*g*Link_1coo;
V2 = m2*g*Link_2coo;
V3 = m3*g*Link_3coo;
V4 = m4*g*Link_4coo;
%Total Potential Energy:
V = V1+V2+V3+V4;
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
Jv2(2,1) = diff(P3y,q1); Jv2(2,2) = diff(P3y,q2); Jv2(2,3) = diff(P3y,q3); Jv2(2,4) = diff(P3x,q4);
Jv2(3,1) = diff(P3z,q1); Jv2(3,2) = diff(P3z,q2); Jv2(3,3) = diff(P3z,q3); Jv2(3,4) = diff(P3x,q4);

Jv3(1,1) = diff(P4x,q1); Jv3(1,2) = diff(P4x,q2); Jv3(1,3) = diff(P4x,q3); Jv3(1,4) = diff(P4x,q4);
Jv3(2,1) = diff(P4y,q1); Jv3(2,2) = diff(P4y,q2); Jv3(2,3) = diff(P4y,q3); Jv3(2,4) = diff(P4x,q4);
Jv3(3,1) = diff(P4z,q1); Jv3(3,2) = diff(P4z,q2); Jv3(3,3) = diff(P4z,q3); Jv3(3,4) = diff(P4x,q4);

Jv4(1,1) = diff(P5x,q1); Jv4(1,2) = diff(P5x,q2); Jv4(1,3) = diff(P5x,q3); Jv4(1,4) = diff(P5x,q4);
Jv4(2,1) = diff(P5y,q1); Jv4(2,2) = diff(P5y,q2); Jv4(2,3) = diff(P5y,q3); Jv4(2,4) = diff(P5x,q4);
Jv3(3,1) = diff(P5z,q1); Jv4(3,2) = diff(P5z,q2); Jv4(3,3) = diff(P5z,q3); Jv4(2,4) = diff(P5x,q4);
%Jacobian Matrix for Rotational Part
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
% LAGRANGIAN TERMS
M = simplify((m1*transpose(Jv1)*Jv1) + (transpose(Jw1)*transpose(R12)*I1*R12*Jw1)...
    + (m2*transpose(Jv2)*Jv2) + (transpose(Jw2)*transpose(R23)*I2*R23*Jw2) ...
    + (m3*transpose(Jv3)*Jv3) + (transpose(Jw3)*transpose(R34)*I3*R34*Jw3)...
    + (m4*transpose(Jv4)*Jv4) + (transpose(Jw4)*transpose(R45)*I4*R45*Jw4));

dM = simplify(diff(M,q1)*dq1 + diff(M,q2)*dq2 + diff(M,q3)*dq3) + diff(M,q4)*dq4;
Ma = -0.5*dq'*M*dq; 
dMa = simplify([diff(Ma,q1); diff(Ma,q2); diff(Ma,q3); diff(Ma,q4)]);
Cc = simplify(dM*dq + M*ddq);
Gr = m1*Jv1'*Grav+m2*Jv2'*Grav + m3*Jv3'*Grav + m4*Jv4'*Grav;
T = simplify(Cc-dMa+Gr);
%Torque Calculation for each actuator:
T1 = T(1);
T2 = T(2);
T3 = T(3);
T4 = T(4);



    