
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

Links(1) = Link('theta' , pi/2, 'a' , 0 , 'alpha' , pi/2, 'qlim' , [0 0]); % dummy

Links(2) = Revolute('d', 0, 'a' , 0 , 'alpha' , 0 ); % Joint 1

Links(3) = Prismatic('theta', 0 , 'a' , 0.156051 , 'qlim' ,[0 0] , 'offset' , 0.156051); % The elbow and pipes.

Links(4) = Prismatic('theta', pi/2 , 'alpha' ,pi/2, 'a' , 0 , 'qlim' ,[0 0] , 'offset' , 0); % Rotate axis.

Links(5) = Revolute('d', 0 , 'a' , 0 , 'alpha' , 0 ); % Joint 2

Links(6) = Prismatic('theta', 0 , 'a' , 0.156051 , 'qlim' ,[0 0] , 'offset' , 0.156051); % The elbow and pipes.

Links(7) = Prismatic('theta', pi/2 , 'alpha' ,pi/2, 'a' , 0 , 'qlim' ,[0 0] , 'offset' , 0); % Rotate axis.

Links(8) = Revolute('d', 0, 'a' , 0 , 'alpha' , 0 ); % Joint 3

Links(9) = Prismatic('theta', 0 , 'a' , 0.156051 , 'qlim' ,[0 0] , 'offset' , 0.156051); % The elbow and pipes.

Links(10) = Prismatic('theta', pi/2 , 'alpha' ,pi/2, 'a' , 0 , 'qlim' ,[0 0] , 'offset' , 0); % Rotate axis.

Links(11) = Revolute('d', 0, 'a' , 0 , 'alpha' , 0 ); % Joint 3

Links(12) = Prismatic('theta', 0 , 'a' , 0 , 'qlim' ,[0 0] , 'offset' , 0.041); % The elbow and pipes.


q = [0 pi/2 0 0 0 0 0 0 0 0 0 0];

%Links(7) = Prismatic('theta', 0 , 'a' , 0.156051 , 'qlim' ,[0 0] , 'offset' , 0.156051);
%Links(8) = Prismatic('theta', pi/2 , 'alpha' ,pi/2, 'a' , 0 , 'qlim' ,[0 0] , 'offset' , 0);


%{    
Links(5) = Revolute('d', 0.156051, 'a' , 0.156051 , 'alpha' , pi/2);

Links(6) = Revolute('d', 0.156051, 'a' , 0 , 'alpha' , 0 );

Links(7) = Prismatic( 'theta', pi/2 ,'a' , 0.041 , 'offset' , 0.5, 'qlim', [0 0]);
%}
robot = SerialLink(Links, 'base' , eye(4));

robot.name = 'OzURobot';


%robot.links.update()
%tau_g = gravload(robot, q)

robot.teach(q)

