
%Different:
clear all
clc
close all
dhtable= [0         0.156051    0.156051     pi/2       0;...  
          0         0.156051    0.156051     pi/2       0;...
          0         0.156051    0.156051     -pi/2      0;...
          0         0.100       0         	 pi/2       0 ];%buradaki offset

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

%Real Joints -> 2, 5 , 8 , 11

% Axis Rotation
Links(1) = Link('theta' , pi/2, 'a' , 0 , 'alpha' , pi/2, 'qlim' , [0 0], 'm' , 0 , 'r' , [0; 0; 0] , 'I', zeros(3)); 

% Joint 1
Links(2) = Revolute('d', 0, 'a' , 0 , 'alpha' , 0 , 'm' , 0 , 'r' , [0; 0; 0] , 'I', zeros(3), 'offset', pi/2);

% The first link.
Links(3) = Prismatic('theta', 0 , 'a' , 0.156051 , 'qlim' ,[0 0] , 'offset' , 0.156051 , 'm' , 0.6 , 'r' , [0.0625; 0 ; 0.0335], 'I', I1); 

%Axis Rotation
Links(4) = Prismatic('theta', pi/2 , 'alpha' ,pi/2, 'a' , 0 , 'qlim' ,[0 0] , 'm' , 0 , 'r' , [0; 0; 0] , 'I', zeros(3)); % Rotate axis.

%Joint 2
Links(5) = Revolute('d', 0 , 'a' , 0 , 'alpha' , 0 ,'m' , 0 , 'r' , [0; 0; 0] , 'I', zeros(3)); % Joint 2

% The second link.
Links(6) = Prismatic('theta', 0 , 'a' , 0.156051 , 'qlim' ,[0 0] , 'offset' , 0.156051, 'm' , 0.6 , 'r' , [0 ; -0.0325 ; 0.0625],'I', I2); 

%Axis Rotation
Links(7) = Prismatic('theta', pi/2 , 'alpha' ,pi/2, 'a' , 0 , 'qlim' ,[0 0] , 'm' , 0 , 'r' , [0; 0; 0] , 'I', zeros(3)); % Rotate axis.

% Joint 3
Links(8) = Revolute('d', 0, 'a' , 0 , 'alpha' , 0 ); 

% The third link
Links(9) = Prismatic('theta', 0 , 'a' , 0.156051 , 'qlim' ,[0 0] , 'offset' , 0.156051, 'm' , 0.6 , 'r' , [0.0625; -0.0335 ; 0],'I', I3); 

%Axis Rotation
Links(10) = Prismatic('theta', pi/2 , 'alpha' ,pi/2, 'a' , 0 , 'qlim' ,[0 0] ,'m' , 0 , 'r' , [0; 0; 0] , 'I', zeros(3)); 

% Joint 3
Links(11) = Revolute('d', 0, 'a' , 0 , 'alpha' , 0 ); 

% The fourth link
Links(12) = Prismatic('theta', 0 , 'a' , 0 , 'qlim' ,[0 0] , 'offset' , 0.041, 'm' , 0.225 , 'r' , [0.010; 0 ; 0],'I', I4); 


%Links(7) = Prismatic('theta', 0 , 'a' , 0.156051 , 'qlim' ,[0 0] , 'offset' , 0.156051);
%Links(8) = Prismatic('theta', pi/2 , 'alpha' ,pi/2, 'a' , 0 , 'qlim' ,[0 0] , 'offset' , 0);


%{    
Links(5) = Revolute('d', 0.156051, 'a' , 0.156051 , 'alpha' , pi/2);

Links(6) = Revolute('d', 0.156051, 'a' , 0 , 'alpha' , 0 );

Links(7) = Prismatic( 'theta', pi/2 ,'a' , 0.041 , 'offset' , 0.5, 'qlim', [0 0]);
%}
robot = SerialLink(Links, 'base' , eye(4));

robot.name = 'MISA';

%Real Joints -> 2, 5 , 8 , 11
syms q1 q2 q3 q4 real
q = [0 0 0 0 0 0 0 0 0 0 0 0];

%robot.links.update()

Q1 = [];
Q2 = [];
Q3 = [];
Q4 = [];
X = [];

for i = 0:0.1:2*pi
    q = [0 i 0 0 0 0 0 0 0 0 0 0];
    tau_g = gravload(robot, q);
    Q1 = [Q1 , tau_g(2)];
    Q2 = [Q2 , tau_g(5)];
    Q3 = [Q3 , tau_g(8)];
    Q4 = [Q4 , tau_g(11)];
    X = [X , i];
end
hold on
plot(X,Q1,'LineWidth',3);
plot(X,Q2,'LineWidth',3);
plot(X,Q3,'LineWidth',3);
plot(X,Q4,'LineWidth',3);
title('Gravity compensation values for sweeping q1');
grid on;
xlabel('q1 angle(rad)');
ylabel('Torque(N/m)');
legend('Joint 1' , 'Joint 2', 'Joint 3','Joint 4');
hold off
