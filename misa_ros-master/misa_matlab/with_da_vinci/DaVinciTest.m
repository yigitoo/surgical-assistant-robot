
q5 = 0;
q6 = 0;
q7 = 0;
q8 = 0;

m1_off = pi/4;
m2_off = pi/4;
m3_off = pi/4;
m4_off = pi/4;

TDaVinci = [[0 0.65 1 m1_off]; 
            [0 0.65 1 m2_off];
            [0.75 0 0 m3_off];
            [0 1 0 m4_off]];
        
RotateDaVinci = inv(TDaVinci);
        
T4 = RotateDaVinci * [q5 0 0 0]';
T5 = RotateDaVinci * [0 q6 0 0]';
T6 = RotateDaVinci * [0 0 q7 0]';
T7 = RotateDaVinci * [0 0 0 q8]';

t = 5;
res = 100;
dt = 5 / res;

time = 0;
while time < t
    
    q5 = q5 + 0.01;
    T4 = RotateDaVinci * [q5; 0; 0; 0];
    T5 = RotateDaVinci * [0; q6; 0 ;0];
    T6 = RotateDaVinci * [0; 0; q7; 0];
    T7 = RotateDaVinci * [0; 0; 0; q8];
    T4
    T05 = T4*T5;
    T06 = T05*T6;
    T07 = T06*T7;
    
    time = time + dt;
end

