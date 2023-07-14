
function TE = ForwardKinematics()
syms q0 q1 q2 q3 q4 q5 q6 q7 q8 real
syms q0d q1d q2d q3d q4d real
syms d0 d1 d2 d3 d4 d5 d6 d7 real;
syms lim0

m1_off = pi;
m2_off = pi;
m3_off = pi;
m4_off = pi;

%{
d1 = 0.100;
d2 = 0.156051;
d3 = 0.156051;
d4 = 0.042;
%}

T0 = [RotateRoll(q1) ,[d1; 0; 0];
	 0 0 0 1];

T1 = [RotateYaw(q2) ,[d2; 0 ; 0];
	 0 0 0 1];

T2 = [RotateYaw(q3) ,[d3; 0; 0];
	 0 0 0 1];

T3 = [RotateRoll(q4) ,[d4; 0; 0];
	 0 0 0 1];

TDaVinci = [[0 0.65 1 m1_off]; 
            [0 0.65 1 m2_off];
            [0.75 0 0 m3_off];
            [0 1 0 m4_off]];
        
RotateDaVinci = inv(TDaVinci);
        
T4 = RotateDaVinci * [q5 0 0]';
T5 = RotateDaVinci * [0 q6 0]';
T6 = RotateDaVinci * [0 0 q7]';


T01 = T0*T1;
T02 = T01*T2;
T03 = T02*T3;
T04 = T03*T4;
T05 = T04*T5;
T06 = T05*T6;
T07 = T06*T7;

TE = T07;
%matlabFunction(TE,'File', 'NumericForwardKinematics')
end



