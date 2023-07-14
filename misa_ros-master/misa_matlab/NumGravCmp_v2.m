function Gr = NumGravCmp_v2(q1,q2,q3)
%NUMGRAVCMP_V1
%    GR = NUMGRAVCMP_V1(Q1,Q2,Q3)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    13-Jun-2019 02:48:23

t2 = cos(q1);
t3 = cos(q2);
t4 = cos(q3);
t5 = sin(q1);
t6 = sin(q2);
t7 = sin(q3);
Gr = [t5.*5.27303588196+t2.*t3.*3.0388911672-t5.*t7.*9.6404732491e-1-t2.*t4.*t6.*9.6404732491e-1;t5.*t6.*(-3.0388911672)-t3.*t4.*t5.*9.6404732491e-1;t2.*t4.*9.6404732491e-1+t5.*t6.*t7.*9.6404732491e-1;0.0];