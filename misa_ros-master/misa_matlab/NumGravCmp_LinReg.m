function Gr = NumGravCmp_LinReg(Le,g,m1,m2,m3,m4,payload,q1,q2,q3,r4x,r_long,r_short)
%NUMGRAVCMP_LINREG
%    GR = NUMGRAVCMP_LINREG(LE,G,M1,M2,M3,M4,PAYLOAD,Q1,Q2,Q3,R4X,R_LONG,R_SHORT)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    17-Aug-2019 19:35:26

t2 = cos(q1);
t3 = cos(q2);
t4 = cos(q3);
t5 = sin(q1);
t6 = sin(q2);
t7 = sin(q3);
t8 = m4+payload;
t9 = t2.*t4;
t10 = t5.*t7;
t11 = Le.*t5.*2.0;
t12 = t6.*t10;
t13 = t6.*t9;
t14 = t9+t12;
t15 = t10+t13;
Gr = [g.*t8.*(t11-Le.*t15-r4x.*t15+Le.*t2.*t3.*2.0)+g.*m3.*(t11-r_short.*t15+Le.*t2.*t3+r_long.*t2.*t3)+g.*m2.*(Le.*t5+r_long.*t5+r_short.*t2.*t3)+g.*m1.*r_short.*t5;-g.*m3.*(Le.*t5.*t6+r_long.*t5.*t6+r_short.*t3.*t4.*t5)-g.*t8.*(t6.*t11+Le.*t3.*t4.*t5+r4x.*t3.*t4.*t5)-g.*m2.*r_short.*t5.*t6;g.*t8.*(Le.*t14+r4x.*t14)+g.*m3.*r_short.*t14;0.0];