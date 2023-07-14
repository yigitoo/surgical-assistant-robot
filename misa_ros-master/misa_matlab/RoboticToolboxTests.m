
clear all
clc
close all

dhtable= [0         0.08        0            0          0;...  
          0         0.156051    0.156051     pi/2       0;...
          pi/2      0           0            pi/2       0;...
          0         0.156051    0.156051     -pi/2      0;...
          0         0.158051    0         	 pi/2       0 ];%buradaki offset

Le = 0.156051;
L0 = 0.08;

Links(1) = Revolute('alpha', -pi , 'd' , L0 , 'a', Le);
Links(2) = Revolute('alpha' , pi/2 , 'd' , Le, 'a' , Le);
Links(3) = Prismatic( 'alpha', pi/2 , 'a' , 0.041 + 1 ,'qlim', [0 0]);
Links(4) = Revolute('alpha' , pi/2, 'd' , Le, 'a' , Le);
Links(5) = Revolute('alpha' , pi/2, 'd' , Le, 'a' , Le);


baseFrame = [RotatePitch(pi/2),[0; 0; 0];
             0 0 0 1];

robot = SerialLink(Links , 'base' , baseFrame);
robot.teach([pi 0 0 0]);

%{
t = 5
for( k = 0:0.1:t)
    Links(3) = Revolute('d', 0.156051,...
    'a' , 0.156051, ...
    'alpha' , pi/2 , ...
    'offset' , k );
    robot = SerialLink(Links, 'base', baseFrame)
    robot.plot([0,0,0,0]);
end
%}