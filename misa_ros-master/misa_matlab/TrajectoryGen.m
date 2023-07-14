% Knee Surgery Robot Trajectory Generation:

clc
clear all

% Knee Surgery Robot Link Lengths
l0 = 0.5;
l1 = 0.35;
l2 = 0.14;
l3 = 0.14;
l4 = 0.108;
l5 = 0.1;
l6 = 0.05;
h1 = 0.3;
h2 = 0.08;
g = 9.81;

I0 = 0.002;
I1 = 0.003;
I2 = 0.001;
I3 = 0.002;
I4 = 0.004;
I5 = 0.005;
I6 = 0.001;

m0 = 1;
m1 = 0.5;
m2 = 1.2;
m3 = 1.3;
m4 = 1.1;
m5 = 1.5;
m6 = 0.2;


tstart = 0.0;
tend = 20;
tsim = tend - tstart;
dt = 0.001;
Dn = tsim/dt + 1;
f = 0.2;

DataStart = tstart/dt + 1;
temp = DataStart-1;

% Trajectory Generation:

for(i=1:Dn)
    t(i,1) = tstart + (i-1)*dt;
    
    q0(i,1) = 0.000001;
    q1(i,1) = 0.000001;
    q2(i,1) = 0.000001;
    q3(i,1) = (45*pi/180);
    h2(i,1) = 0.000001;
    
    [xPos(i), xVel(i), xAcc(i)] = FuncPoly5th(t(i), 5, tend, 0.05,  0,  0,  0.56,  0,  0, dt);
    [zPos(i), zVel(i), zAcc(i)] = FuncPoly5th(t(i), 5, tend, 0.005, 0, 0, 0.007, 0, 0, dt);
    [yPos(i), yVel(i), yAcc(i)] = FuncPoly5th(t(i), 5, tend,  0.4,  0,  0,  0.3,  0,  0, dt);
    [alpha(i), alphaVel(i), alphaAcc(i)] = FuncPoly5th(t(i), 0, tend, 0, 0, 0, 0, 0, 0, dt);
    [beta(i), betaVel(i), betaAcc(i)] = FuncPoly5th(t(i), 0, tend, 0, 0, 0, 0, 0, 0, dt);



% Inverse Kinematics Starts     

 if i>1
     
J_inv = [                               0,                               0,                                                                                                                                                                                                                                                                                         cos(q0(i) + q1(i) + q2(i))/(l6*cos(q3(i))),                                                                   1,                                                0;
 -sin(q0(i) + q1(i) + q2(i))/(l4*cos(q2(i))),  cos(q0(i) + q1(i) + q2(i))/(l4*cos(q2(i))),                                                                                                -(l4*cos(q0(i) + q1(i) + 2*q2(i)) + l6*cos(q0(i) + q1(i) + q2(i) + q3(i)) + l4*cos(q0(i) + q1(i)) + l3*cos(q0(i)) + l3*cos(q0(i) + 2*q1(i) + 2*q2(i)) + l6*cos(q0(i) + q1(i) + q2(i) - q3(i)) - 2*h2(i)*cos(q0(i) + q1(i) + q2(i)) + 2*l5*cos(q0(i) + q1(i) + q2(i)))/(2*l4*l6*cos(q2(i))*cos(q3(i))), -(l5 - h2(i) + l3*cos(q1(i) + q2(i)) + l4*cos(q2(i)) + l6*cos(q3(i)))/(l4*cos(q2(i))),             -(l5 - h2(i) + l6*cos(q3(i)))/(l4*cos(q2(i)));
  sin(q0(i) + q1(i) + q2(i))/(l4*cos(q2(i))), -cos(q0(i) + q1(i) + q2(i))/(l4*cos(q2(i))),                                                                                                 (l4*cos(q0(i) + q1(i) + 2*q2(i)) + l6*cos(q0(i) + q1(i) + q2(i) + q3(i)) + l4*cos(q0(i) + q1(i)) + l3*cos(q0(i)) + l3*cos(q0(i) + 2*q1(i) + 2*q2(i)) + l6*cos(q0(i) + q1(i) + q2(i) - q3(i)) - 2*h2(i)*cos(q0(i) + q1(i) + q2(i)) + 2*l5*cos(q0(i) + q1(i) + q2(i)))/(2*l4*l6*cos(q2(i))*cos(q3(i))),  (l5 - h2(i) + l3*cos(q1(i) + q2(i)) + l4*cos(q2(i)) + l6*cos(q3(i)))/(l4*cos(q2(i))), (l5 - h2(i) + l4*cos(q2(i)) + l6*cos(q3(i)))/(l4*cos(q2(i)));
                               0,                               0,                                                                                                                                                                                                                                                                                                        -1/(l6*cos(q3(i))),                                                                   0,                                                0;
           -cos(q0(i) + q1(i))/cos(q2(i)),           -sin(q0(i) + q1(i))/cos(q2(i)), -(l6*sin(q0(i) + q1(i) + 2*q2(i) - q3(i)) - 2*h2(i)*sin(q0(i) + q1(i) + 2*q2(i)) - 2*l3*sin(q0(i) + 2*q1(i) + q2(i)) + 2*l5*sin(q0(i) + q1(i) + 2*q2(i)) - l6*sin(q0(i) + q1(i) - q3(i)) + 2*h2(i)*sin(q0(i) + q1(i)) + 2*l3*sin(q0(i) + q2(i)) - 2*l5*sin(q0(i) + q1(i)) - 2*l6*sin(q2(i) + q3(i)) + l6*sin(q0(i) + q1(i) + 2*q2(i) + q3(i)) + 2*l6*sin(q2(i) - q3(i)) - l6*sin(q0(i) + q1(i) + q3(i)))/(4*l6*cos(q2(i))*cos(q3(i))), (h2(i)*sin(q2(i)) + l3*sin(q1(i)) - l5*sin(q2(i)) - l6*cos(q3(i))*sin(q2(i)))/cos(q2(i)),        -(sin(q2(i))*(l5 - h2(i) + l6*cos(q3(i))))/cos(q2(i))];
 

   P_diff(1,1) = xPos(i)-xPos(i-1);
   P_diff(2,1) = yPos(i)-yPos(i-1);
   P_diff(3,1) = zPos(i)-zPos(i-1);
   P_diff(4,1) = alpha(i)-alpha(i-1);
   P_diff(5,1) = beta(i)-beta(i-1);
   
        
   Q_diff = (J_inv)*(P_diff); 
        
   q0(i) = q0(i-1) + Q_diff(1,1);
   q1(i) = q1(i-1) + Q_diff(2,1);
   q2(i) = q2(i-1) + Q_diff(3,1);
   q3(i) = q3(i-1) + Q_diff(4,1);
   h2(i) = h2(i-1) + Q_diff(5,1);
   
    else
    q0(i) = q0(1);
    q1(i) = q1(1);
    q2(i) = q2(1);
    q3(i) = q3(1);
    h2(i) = h2(1);   
   
 end
 
   if i>1
   dq0(i) = (q0(i)-q0(i-1))/dt;
   dq1(i) = (q1(i)-q1(i-1))/dt;
   dq2(i) = (q2(i)-q2(i-1))/dt;
   dq3(i) = (q3(i)-q3(i-1))/dt;
   dh2(i) = (h2(i)-h2(i-1))/dt;
   else
    dq0(i) = 0;
    dq1(i) = 0;
    dq2(i) = 0;
    dq3(i) = 0;
    dh2(i) = 0;
    
   end
 
 if i>2
       
   ddq0(i) = (q0(i)-(2*q0(i-1))+q0(i-2))/dt^2;    
   ddq1(i) = (q1(i)-(2*q1(i-1))+q1(i-2))/dt^2;
   ddq2(i) = (q2(i)-(2*q2(i-1))+q2(i-2))/dt^2;
   ddq3(i) = (q3(i)-(2*q3(i-1))+q3(i-2))/dt^2;
   ddh2(i) = (h2(i)-(2*h2(i-1))+h2(i-2))/dt^2;
    
   else
    
    ddq0(i) = 0;
    ddq1(i) = 0;
    ddq2(i) = 0;
    ddq3(i) = 0;
    ddh2(i) = 0;
    
 end
 


% ddq0(19999) = ddq0(19998);
% ddq1(19999) = ddq1(19998);
% ddq2(19999) = ddq2(19998);
% ddq3(19999) = ddq3(19998);
% ddh2(19999) = ddh2(19998);
% 
% ddq0(20000) = ddq0(19999);
% ddq1(20000) = ddq1(19999);
% ddq2(20000) = ddq2(19999);
% ddq3(20000) = ddq3(19999);
% ddh2(20000) = ddh2(19999);
% 
if i>20000
ddq0(i) = ddq0(20000);
ddq1(i) = ddq1(20000);
ddq2(i) = ddq2(20000);
ddq3(i) = ddq3(20000);
ddh2(i) = ddh2(20000);
end
 
%Inverse Kinematics Ends


T1(i) = ddq0(i)*(I0 + I2 + I3 + I4 + I5 + (l2^2*m2)/4 + l2^2*m3 + l2^2*m4 + l2^2*m5 + l2^2*m6) + ddq1(i)*(I2 + I3 + I4 + I5 + (l2*l3*m3*cos(q0(i) - q1(i)))/2 + l2*l3*m4*cos(q0(i) - q1(i)) + l2*l3*m5*cos(q0(i) - q1(i)) + l2*l3*m6*cos(q0(i) - q1(i))) + ddq2(i)*(I3 + I4 + I5 + (l2*l4*m4*cos(q0(i) - q2(i)))/2 + l2*l4*m5*cos(q0(i) - q2(i)) + l2*l4*m6*cos(q0(i) - q2(i))) - (dq0(i)*l2*(dq3(i)*h2(i)*m5*sin(q0(i) + q3(i)) - 2*dh2(i)*m6*cos(q0(i) - q3(i)) - dh2(i)*m5*cos(q0(i) + q3(i)) - 2*dh2(i)*m6*cos(q0(i) + q3(i)) - dh2(i)*m5*cos(q0(i) - q3(i)) + 2*dq3(i)*h2(i)*m6*sin(q0(i) + q3(i)) - dq3(i)*l5*m5*sin(q0(i) + q3(i)) - 2*dq3(i)*l5*m6*sin(q0(i) + q3(i)) - dq3(i)*h2(i)*m5*sin(q0(i) - q3(i)) - 2*dq3(i)*h2(i)*m6*sin(q0(i) - q3(i)) + 2*dq1(i)*l3*m3*sin(q0(i) - q1(i))  + 4*dq1(i)*l3*m4*sin(q0(i) - q1(i)) + 4*dq1(i)*l3*m5*sin(q0(i) - q1(i)) + 4*dq1(i)*l3*m6*sin(q0(i) - q1(i)) + 2*dq2(i)*l4*m4*sin(q0(i) - q2(i)) + 4*dq2(i)*l4*m5*sin(q0(i) - q2(i)) + 4*dq2(i)*l4*m6*sin(q0(i) - q2(i)) + dq3(i)*l5*m5*sin(q0(i) - q3(i)) + 2*dq3(i)*l5*m6*sin(q0(i) - q3(i))))/4 + (dh2(i)*l2*(dq0(i)*cos(q0(i))*cos(q3(i)) - dq3(i)*sin(q0(i))*sin(q3(i)))*(m5 + 2*m6))/2 - (dq3(i)*l2*(m5 + 2*m6)*(dh2(i)*sin(q0(i))*sin(q3(i)) + dq0(i)*h2(i)*cos(q0(i))*sin(q3(i)) + dq3(i)*h2(i)*cos(q3(i))*sin(q0(i)) - dq0(i)*l5*cos(q0(i))*sin(q3(i)) - dq3(i)*l5*cos(q3(i))*sin(q0(i))))/2 + (ddh2(i)*l2*cos(q3(i))*sin(q0(i))*(m5 + 2*m6))/2 - (ddq3(i)*l2*sin(q0(i))*sin(q3(i))*(h2(i) - l5)*(m5 + 2*m6))/2 - (dq1(i)*l2*l3*sin(q0(i) - q1(i))*(dq0(i) - dq1(i))*(m3 + 2*m4 + 2*m5 + 2*m6))/2 - (dq2(i)*l2*l4*sin(q0(i) - q2(i))*(dq0(i) - dq2(i))*(m4 + 2*m5 + 2*m6))/2;                                                                                             
T2(i) = ddq1(i)*(I2 + I3 + I4 + I5 + (l3^2*m3)/4 + l3^2*m4 + l3^2*m5 + l3^2*m6) + ddq0(i)*(I2 + I3 + I4 + I5 + (l2*l3*m3*cos(q0(i) - q1(i)))/2 + l2*l3*m4*cos(q0(i) - q1(i)) + l2*l3*m5*cos(q0(i) - q1(i)) + l2*l3*m6*cos(q0(i) - q1(i))) + ddq2(i)*(I3 + I4 + I5 + (l3*l4*m4*cos(q1(i) - q2(i)))/2 + l3*l4*m5*cos(q1(i) - q2(i)) + l3*l4*m6*cos(q1(i) - q2(i))) + (dq1(i)*l3*(dh2(i)*m5*cos(q1(i) - q3(i)) + 2*dh2(i)*m6*cos(q1(i) - q3(i)) + dh2(i)*m5*cos(q1(i) + q3(i)) + 2*dh2(i)*m6*cos(q1(i) + q3(i)) - dq3(i)*h2(i)*m5*sin(q1(i) + q3(i)) - 2*dq3(i)*h2(i)*m6*sin(q1(i) + q3(i)) + dq3(i)*l5*m5*sin(q1(i) + q3(i)) + 2*dq3(i)*l5*m6*sin(q1(i) + q3(i)) + dq3(i)*h2(i)*m5*sin(q1(i) - q3(i)) + 2*dq3(i)*h2(i)*m6*sin(q1(i) - q3(i)) + 2*dq0(i)*l2*m3*sin(q0(i) - q1(i)) + 4*dq0(i)*l2*m4*sin(q0(i) - q1(i)) + 4*dq0(i)*l2*m5*sin(q0(i) - q1(i)) + 4*dq0(i)*l2*m6*sin(q0(i) - q1(i)) - 2*dq2(i)*l4*m4*sin(q1(i) - q2(i)) - 4*dq2(i)*l4*m5*sin(q1(i) - q2(i)) - 4*dq2(i)*l4*m6*sin(q1(i) - q2(i)) - dq3(i)*l5*m5*sin(q1(i) - q3(i)) - 2*dq3(i)*l5*m6*sin(q1(i) - q3(i))))/4 + (dh2(i)*l3*(dq1(i)*cos(q1(i))*cos(q3(i)) - dq3(i)*sin(q1(i))*sin(q3(i)))*(m5 + 2*m6))/2 - (dq3(i)*l3*(m5 + 2*m6)*(dh2(i)*sin(q1(i))*sin(q3(i)) + dq1(i)*h2(i)*cos(q1(i))*sin(q3(i)) + dq3(i)*h2(i)*cos(q3(i))*sin(q1(i)) - dq1(i)*l5*cos(q1(i))*sin(q3(i)) - dq3(i)*l5*cos(q3(i))*sin(q1(i))))/2 + (ddh2(i)*l3*cos(q3(i))*sin(q1(i))*(m5 + 2*m6))/2 - (ddq3(i)*l3*sin(q1(i))*sin(q3(i))*(h2(i) - l5)*(m5 + 2*m6))/2 - (dq0(i)*l2*l3*sin(q0(i) - q1(i))*(dq0(i) - dq1(i))*(m3 + 2*m4 + 2*m5 + 2*m6))/2 - (dq2(i)*l3*l4*sin(q1(i) - q2(i))*(dq1(i) - dq2(i))*(m4 + 2*m5 + 2*m6))/2;                                                                                                                                                                                                                                                                                                                                                                 
T3(i) = ddq2(i)*(I3 + I4 + I5 + (l4^2*m4)/4 + l4^2*m5 + l4^2*m6) + ddq0(i)*(I3 + I4 + I5 + (l2*l4*m4*cos(q0(i) - q2(i)))/2 + l2*l4*m5*cos(q0(i) - q2(i)) + l2*l4*m6*cos(q0(i) - q2(i))) + ddq1(i)*(I3 + I4 + I5 + (l3*l4*m4*cos(q1(i) - q2(i)))/2 + l3*l4*m5*cos(q1(i) - q2(i)) + l3*l4*m6*cos(q1(i) - q2(i))) + (dq2(i)*l4*(dh2(i)*m5*cos(q2(i) - q3(i)) + 2*dh2(i)*m6*cos(q2(i) - q3(i)) + dh2(i)*m5*cos(q2(i) + q3(i)) + 2*dh2(i)*m6*cos(q2(i) + q3(i)) - dq3(i)*h2(i)*m5*sin(q2(i) + q3(i)) - 2*dq3(i)*h2(i)*m6*sin(q2(i) + q3(i)) + dq3(i)*l5*m5*sin(q2(i) + q3(i)) + 2*dq3(i)*l5*m6*sin(q2(i) + q3(i)) + dq3(i)*h2(i)*m5*sin(q2(i) - q3(i)) + 2*dq3(i)*h2(i)*m6*sin(q2(i) - q3(i)) + 2*dq0(i)*l2*m4*sin(q0(i) - q2(i)) + 4*dq0(i)*l2*m5*sin(q0(i) - q2(i)) + 4*dq0(i)*l2*m6*sin(q0(i) - q2(i)) + 2*dq1(i)*l3*m4*sin(q1(i) - q2(i)) + 4*dq1(i)*l3*m5*sin(q1(i) - q2(i)) + 4*dq1(i)*l3*m6*sin(q1(i) - q2(i)) - dq3(i)*l5*m5*sin(q2(i) - q3(i)) - 2*dq3(i)*l5*m6*sin(q2(i) - q3(i))))/4 + (dh2(i)*l4*(dq2(i)*cos(q2(i))*cos(q3(i)) - dq3(i)*sin(q2(i))*sin(q3(i)))*(m5 + 2*m6))/2 - (dq3(i)*l4*(m5 + 2*m6)*(dh2(i)*sin(q2(i))*sin(q3(i)) + dq2(i)*h2(i)*cos(q2(i))*sin(q3(i)) + dq3(i)*h2(i)*cos(q3(i))*sin(q2(i)) - dq2(i)*l5*cos(q2(i))*sin(q3(i)) - dq3(i)*l5*cos(q3(i))*sin(q2(i))))/2 + (ddh2(i)*l4*cos(q3(i))*sin(q2(i))*(m5 + 2*m6))/2 - (ddq3(i)*l4*sin(q2(i))*sin(q3(i))*(h2(i) - l5)*(m5 + 2*m6))/2 - (dq0(i)*l2*l4*sin(q0(i) - q2(i))*(dq0(i) - dq2(i))*(m4 + 2*m5 + 2*m6))/2 - (dq1(i)*l3*l4*sin(q1(i) - q2(i))*(dq1(i) - dq2(i))*(m4 + 2*m5 + 2*m6))/2;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
T4(i) = ddq3(i)*(I4 + I5 + (h2(i)^2*m5)/4 + h2(i)^2*m6 + (l5^2*m5)/4 + l5^2*m6 - (h2(i)*l5*m5)/2 - 2*h2(i)*l5*m6) - ((m5 + 2*m6)*(dh2(i)*sin(q3(i)) + dq3(i)*h2(i)*cos(q3(i)) - dq3(i)*l5*cos(q3(i)))*(dq0(i)*l2*sin(q0(i)) + dq1(i)*l3*sin(q1(i)) + dq2(i)*l4*sin(q2(i))))/2 - (dq0(i)*l2*(m5 + 2*m6)*(dh2(i)*sin(q0(i))*sin(q3(i)) + dq0(i)*h2(i)*cos(q0(i))*sin(q3(i)) + dq3(i)*h2(i)*cos(q3(i))*sin(q0(i)) - dq0(i)*l5*cos(q0(i))*sin(q3(i)) - dq3(i)*l5*cos(q3(i))*sin(q0(i))))/2 - (dq1(i)*l3*(m5 + 2*m6)*(dh2(i)*sin(q1(i))*sin(q3(i)) + dq1(i)*h2(i)*cos(q1(i))*sin(q3(i)) + dq3(i)*h2(i)*cos(q3(i))*sin(q1(i)) - dq1(i)*l5*cos(q1(i))*sin(q3(i)) - dq3(i)*l5*cos(q3(i))*sin(q1(i))))/2 - (dq2(i)*l4*(m5 + 2*m6)*(dh2(i)*sin(q2(i))*sin(q3(i)) + dq2(i)*h2(i)*cos(q2(i))*sin(q3(i)) + dq3(i)*h2(i)*cos(q3(i))*sin(q2(i)) - dq2(i)*l5*cos(q2(i))*sin(q3(i)) - dq3(i)*l5*cos(q3(i))*sin(q2(i))))/2 + (dh2(i)*dq3(i)*(h2(i) - l5)*(m5 + 4*m6))/2 + g*m6*cos(q3(i))*(h2(i) - l5) + g*m5*cos(q3(i))*(h2(i)/2 - l5/2) - (ddq0(i)*l2*sin(q0(i))*sin(q3(i))*(h2(i) - l5)*(m5 + 2*m6))/2 - (ddq1(i)*l3*sin(q1(i))*sin(q3(i))*(h2(i) - l5)*(m5 + 2*m6))/2 - (ddq2(i)*l4*sin(q2(i))*sin(q3(i))*(h2(i) - l5)*(m5 + 2*m6))/2;
T5(i) = ddh2(i)*(m5/4 + m6) + (g*m5*sin(q3(i)))/2 + g*m6*sin(q3(i)) + (dq3(i)^2*h2(i)*m5)/4 + dq3(i)^2*h2(i)*m6 - (dq3(i)^2*l5*m5)/4 - dq3(i)^2*l5*m6 + (dq0(i)*l2*(dq0(i)*cos(q0(i))*cos(q3(i)) - dq3(i)*sin(q0(i))*sin(q3(i)))*(m5 + 2*m6))/2 + (dq1(i)*l3*(dq1(i)*cos(q1(i))*cos(q3(i)) - dq3(i)*sin(q1(i))*sin(q3(i)))*(m5 + 2*m6))/2 + (dq2(i)*l4*(dq2(i)*cos(q2(i))*cos(q3(i)) - dq3(i)*sin(q2(i))*sin(q3(i)))*(m5 + 2*m6))/2 + (ddq0(i)*l2*cos(q3(i))*sin(q0(i))*(m5 + 2*m6))/2 + (ddq1(i)*l3*cos(q3(i))*sin(q1(i))*(m5 + 2*m6))/2 + (ddq2(i)*l4*cos(q3(i))*sin(q2(i))*(m5 + 2*m6))/2 - (dq0(i)*dq3(i)*l2*m5*sin(q0(i))*sin(q3(i)))/2 - dq0(i)*dq3(i)*l2*m6*sin(q0(i))*sin(q3(i)) - (dq1(i)*dq3(i)*l3*m5*sin(q1(i))*sin(q3(i)))/2 - dq1(i)*dq3(i)*l3*m6*sin(q1(i))*sin(q3(i)) - (dq2(i)*dq3(i)*l4*m5*sin(q2(i))*sin(q3(i)))/2 - dq2(i)*dq3(i)*l4*m6*sin(q2(i))*sin(q3(i));

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
    rf_P0(i-temp,1) = 0;
    rf_P0(i-temp,2) = 0;
    rf_P0(i-temp,3) = 0;
    
    rf_P1(i-temp,1) = 0;
    rf_P1(i-temp,2) = 0;
    rf_P1(i-temp,3) = l0 - h1 + l1;
    
    rf_P2(i-temp,1) =  l2*cos(q0(i));
    rf_P2(i-temp,2) =  l2*sin(q0(i));
    rf_P2(i-temp,3) =  l0 - h1 + l1;
    
    rf_P3(i-temp,1) =  l2*cos(q0(i)) + l3*cos(q1(i));
    rf_P3(i-temp,2) = l2*sin(q0(i)) + l3*sin(q1(i));
    rf_P3(i-temp,3) = l0 - h1 + l1;
    
    rf_P4(i-temp,1) = l2*cos(q0(i)) + l3*cos(q1(i)) + l4*cos(q2(i));
    rf_P4(i-temp,2) = l2*sin(q0(i)) + l3*sin(q1(i)) + l4*sin(q2(i));
    rf_P4(i-temp,3) =  l0 - h1 + l1;
    
    rf_P5(i-temp,1) = l2*cos(q0(i)) + l3*cos(q1(i)) + l4*cos(q2(i)) - cos(q3(i))*(h2(i) - l5);
    rf_P5(i-temp,2) = l2*sin(q0(i)) + l3*sin(q1(i)) + l4*sin(q2(i));
    rf_P5(i-temp,3) = l0 - h1 + l1 + sin(q3(i))*(h2(i) - l5);
    
    rf_P6(i-temp,1) = l2*cos(q0(i)) + l3*cos(q1(i)) + l4*cos(q2(i)) + l6*cos(q3(i)) - cos(q3(i))*(h2(i) - l5);
    rf_P6(i-temp,2) = l2*sin(q0(i)) + l3*sin(q1(i)) + l4*sin(q2(i));
    rf_P6(i-temp,3) = l0 - h1 + l1 - l6*sin(q3(i)) + sin(q3(i))*(h2(i) - l5);
    
end

figure(1);
set(gcf,'Renderer','OpenGL');
set(gcf, 'Position', [300 200 1200 600]);
RF_P0 = plot3(rf_P0(1,1),rf_P0(1,2),rf_P0(1,3),'o','MarkerSize',10,'MarkerFaceColor','g');
hold on;
RF_P1 = plot3(rf_P1(1,1),rf_P1(1,2),rf_P1(1,3),'o','MarkerSize',10,'MarkerFaceColor','b');
RF_P2 = plot3(rf_P2(1,1),rf_P2(1,2),rf_P2(1,3),'o','MarkerSize',10,'MarkerFaceColor','r');
RF_P3 = plot3(rf_P3(1,1),rf_P3(1,2),rf_P3(1,3),'o','MarkerSize',10,'MarkerFaceColor','m');
RF_P4 = plot3(rf_P4(1,1),rf_P4(1,2),rf_P4(1,3),'o','MarkerSize',10,'MarkerFaceColor','y');
RF_P5 = plot3(rf_P5(1,1),rf_P5(1,2),rf_P5(1,3),'o','MarkerSize',10,'MarkerFaceColor','k');
RF_P6 = plot3(rf_P6(1,1),rf_P6(1,2),rf_P6(1,3),'o','MarkerSize',10,'MarkerFaceColor','c');

RF_D0 = plot3([rf_P0(1,1) rf_P1(1,1)], [rf_P0(1,2) rf_P1(1,2)], [rf_P0(1,3) rf_P1(1,3)],'LineWidth',4,'Color','k');
RF_D1 = plot3([rf_P1(1,1) rf_P2(1,1)], [rf_P1(1,2) rf_P2(1,2)], [rf_P1(1,3) rf_P2(1,3)],'LineWidth',4,'Color','k');
RF_D2 = plot3([rf_P2(1,1) rf_P3(1,1)], [rf_P2(1,2) rf_P3(1,2)], [rf_P2(1,3) rf_P3(1,3)],'LineWidth',4,'Color','k');
RF_D3 = plot3([rf_P3(1,1) rf_P4(1,1)], [rf_P3(1,2) rf_P4(1,2)], [rf_P3(1,3) rf_P4(1,3)],'LineWidth',4,'Color','k');
RF_D4 = plot3([rf_P4(1,1) rf_P5(1,1)], [rf_P4(1,2) rf_P5(1,2)], [rf_P4(1,3) rf_P5(1,3)],'LineWidth',4,'Color','k');
RF_D5 = plot3([rf_P5(1,1) rf_P6(1,1)], [rf_P5(1,2) rf_P6(1,2)], [rf_P5(1,3) rf_P6(1,3)],'LineWidth',4,'Color','k');

xlim([-2 2]);xlabel('X[m]','FontSize',16);
ylim([-2 2]);ylabel('Y[m]','FontSize',16);
zlim([-2 2]);zlabel('Z[m]','FontSize',16);
grid;
    
set(RF_P0,'EraseMode','normal');
set(RF_P1,'EraseMode','normal');
set(RF_P2,'EraseMode','normal');
set(RF_P3,'EraseMode','normal');
set(RF_P4,'EraseMode','normal');
set(RF_P5,'EraseMode','normal');
set(RF_P6,'EraseMode','normal');
set(RF_D0,'EraseMode','normal');
set(RF_D1,'EraseMode','normal');
set(RF_D2,'EraseMode','normal');
set(RF_D3,'EraseMode','normal');
set(RF_D4,'EraseMode','normal');
set(RF_D5,'EraseMode','normal');

i = 1;
while i<=size(rf_P0,1)
    set(RF_P0,'XData',rf_P0(i,1),'YData',rf_P0(i,2),'ZData',rf_P0(i,3));
    set(RF_P1,'XData',rf_P1(i,1),'YData',rf_P1(i,2),'ZData',rf_P1(i,3));
    set(RF_P2,'XData',rf_P2(i,1),'YData',rf_P2(i,2),'ZData',rf_P2(i,3));
    set(RF_P3,'XData',rf_P3(i,1),'YData',rf_P3(i,2),'ZData',rf_P3(i,3));
    set(RF_P4,'XData',rf_P4(i,1),'YData',rf_P4(i,2),'ZData',rf_P4(i,3));
    set(RF_P5,'XData',rf_P5(i,1),'YData',rf_P5(i,2),'ZData',rf_P5(i,3));
    set(RF_P6,'XData',rf_P6(i,1),'YData',rf_P6(i,2),'ZData',rf_P6(i,3));
    
    set(RF_D0,'XData',[rf_P0(i,1) rf_P1(i,1)],'YData',[rf_P0(i,2) rf_P1(i,2)],'ZData',[rf_P0(i,3) rf_P1(i,3)]);
    set(RF_D1,'XData',[rf_P1(i,1) rf_P2(i,1)],'YData',[rf_P1(i,2) rf_P2(i,2)],'ZData',[rf_P1(i,3) rf_P2(i,3)]);
    set(RF_D2,'XData',[rf_P2(i,1) rf_P3(i,1)],'YData',[rf_P2(i,2) rf_P3(i,2)],'ZData',[rf_P2(i,3) rf_P3(i,3)]);
    set(RF_D3,'XData',[rf_P3(i,1) rf_P4(i,1)],'YData',[rf_P3(i,2) rf_P4(i,2)],'ZData',[rf_P3(i,3) rf_P4(i,3)]);
    set(RF_D4,'XData',[rf_P4(i,1) rf_P5(i,1)],'YData',[rf_P4(i,2) rf_P5(i,2)],'ZData',[rf_P4(i,3) rf_P5(i,3)]);
    set(RF_D5,'XData',[rf_P5(i,1) rf_P6(i,1)],'YData',[rf_P5(i,2) rf_P6(i,2)],'ZData',[rf_P5(i,3) rf_P6(i,3)]);
    drawnow;
    i = i+25;
end





