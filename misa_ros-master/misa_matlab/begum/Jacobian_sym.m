clear
clc
syms q1 q2 q3 q4 q5 q6 q7 real
syms dq1 dq2 dq3 dq4 dq5 dq6 dq7 real

Lse = 0.18022;
Lpe = 0.061 + 0.005/2 + 0.119221;

L0x = 0.061;
L1x = Lpe;
L1z = Lpe;
L2y = Lpe;
L2z = Lpe;
L3x = Lse;
L3y = Lse;
L4x = 0.120063;
L5z = 0.485;
L6z = 0.0096;
L7z = 0.01183;

T1 = TransformRoll(q1, [L0x; 0; 0]);
T2 = TransformYaw(q2, [L1x; 0; L1z]);
T3 = TransformPitch(q3, [0; -L2y; L2z]);
T4 = TransformRoll(q4, [L3x; -L3y; 0]);
T5 = TransformYaw(q5, [L4x; 0; 0]);
T6 = TransformPitch(q6, [0; 0; -L5z]);
T7 = TransformRoll(q7, [0; 0; -L6z]);
T8 = TransformNoRot([0; 0; -L7z]);

T = simplify(T1*T2*T3*T4*T5*T6*T7*T8);

Xe = T(1,4);
Ye = T(2,4);
Ze = T(3,4);

R = T(1:3,1:3);

SSM = simplify(doDiff(R)*transpose(R)); % Skew Symmetric Matrix

Wbx = SSM(3,2);
Wby = SSM(1,3);
Wbz = SSM(2,1);

J = sym(eye(7));
J(1,1) = diff(Xe,q1);
J(1,2) = diff(Xe,q2);
J(1,3) = diff(Xe,q3);
J(1,4) = diff(Xe,q4);
J(1,5) = diff(Xe,q5);
J(1,6) = diff(Xe,q6);
J(1,7) = diff(Xe,q7);

J(2,1) = diff(Ye,q1);
J(2,2) = diff(Ye,q2);
J(2,3) = diff(Ye,q3);
J(2,4) = diff(Ye,q4);
J(2,5) = diff(Ye,q5);
J(2,6) = diff(Ye,q6);
J(2,7) = diff(Ye,q7);

J(3,1) = diff(Ze,q1);
J(3,2) = diff(Ze,q2);
J(3,3) = diff(Ze,q3);
J(3,4) = diff(Ze,q4);
J(3,5) = diff(Ze,q5);
J(3,6) = diff(Ze,q6);
J(3,7) = diff(Ze,q7);

J(4,1) = diff(Wbx,dq1);
J(4,2) = diff(Wbx,dq2);
J(4,3) = diff(Wbx,dq3);
J(4,4) = diff(Wbx,dq4);
J(4,5) = diff(Wbx,dq5);
J(4,6) = diff(Wbx,dq6);
J(4,7) = diff(Wbx,dq7);

J(5,1) = diff(Wby,dq1);
J(5,2) = diff(Wby,dq2);
J(5,3) = diff(Wby,dq3);
J(5,4) = diff(Wby,dq4);
J(5,5) = diff(Wby,dq5);
J(5,6) = diff(Wby,dq6);
J(5,7) = diff(Wby,dq7);

J(6,1) = diff(Wbz,dq1);
J(6,2) = diff(Wbz,dq2);
J(6,3) = diff(Wbz,dq3);
J(6,4) = diff(Wbz,dq4);
J(6,5) = diff(Wbz,dq5);
J(6,6) = diff(Wbz,dq6);
J(6,7) = diff(Wbz,dq7);

J = simplify(J);

matlabFunction(J, 'File', 'Jacobian');
