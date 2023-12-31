function TE = ForwardKinematics(q1,q2,q3,q4,q5)
%ForwardKinematics
%    TE = ForwardKinematics(Q1,Q2,Q3,Q4,Q5)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    25-Jul-2023 23:05:35

t2 = cos(q1);
t3 = cos(q2);
t4 = cos(q3);
t5 = cos(q4);
t6 = cos(q5);
t7 = sin(q1);
t8 = sin(q2);
t9 = sin(q3);
t10 = sin(q4);
t11 = sin(q5);
t12 = -q3;
t13 = t2.*t10;
t14 = t7.*t10;
t15 = q2+t12;
t17 = t7.*t8.*t9;
t19 = t2.*t3.*t4;
t20 = t2.*t3.*t9;
t21 = t2.*t4.*t8;
t22 = t3.*t4.*t7;
t23 = t2.*t8.*t9;
t24 = t3.*t7.*t9;
t25 = t4.*t7.*t8;
t16 = cos(t15);
t18 = sin(t15);
t26 = -t21;
t27 = -t25;
t28 = t19+t23;
t29 = t17+t22;
t30 = t20+t26;
t31 = t24+t27;
t32 = t5.*t28;
t33 = t5.*t29;
t34 = -t32;
t35 = t13+t33;
t36 = t14+t34;
mt1 = [-t11.*t30-t6.*t36,-t6.*t35+t11.*t31,-t11.*t16+t5.*t6.*t18,0.0,t5.*t7+t10.*t28,t2.*t5-t10.*t29,t10.*t18,0.0,t6.*t30-t11.*t36,-t6.*t31-t11.*t35,t6.*t16+t5.*t11.*t18,0.0,t20.*(7.0./2.5e+1)-t21.*(7.0./2.5e+1)-t2.*t8.*1.6501e-1+t6.*t30.*6.508e-1-t11.*t36.*6.508e-1,t24.*(-7.0./2.5e+1)+t25.*(7.0./2.5e+1)+t7.*t8.*1.6501e-1-t6.*t31.*6.508e-1-t11.*t35.*6.508e-1];
mt2 = [t3.*1.6501e-1+t16.*(7.0./2.5e+1)+t6.*t16.*6.508e-1+t18.*sin(q4+q5).*3.254e-1-t18.*sin(q4-q5).*3.254e-1+2.13e+2./6.25e+2,1.0];
TE = reshape([mt1,mt2],4,4);
