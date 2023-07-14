
close all
clear all
clc

iter = 0.01;
q1 = 0;
q2 = 0;
q3 = 0;

X = [];
Y = [];
Z = [];

for q2 = 0 : iter : 2*pi

    P = COMTest(q1,q2,q3);

    X = [P(1,1), P(1,2), P(1,3), X];
    Y = [P(2,1), P(2,2), P(2,3), Y];
    Z = [P(3,1), P(3,2), P(3,3), Z];

end

scatter3(X,Y,Z);
