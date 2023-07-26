clear;
clc;

myobj = FunctionsPathPlan;

q1 = 0; q2 = deg2rad(0); q3 = deg2rad(0); q4 = 0; q5 = deg2rad(-90);

TE = ForwardKinematics(q1, q2, q3, q4, q5);
xs = TE(1, 4);
ys = TE(2, 4);
zs = TE(3, 4);
xs
ys
zs

% Sub task 1: Increase x 
commands = [];
positions = [];

commands = [commands; [q1, q2, q3, q4, q5]];
positions= [positions; [xs, ys, zs]];

% For trajectory generation:
% Use the sensor data of q1, q2, q3, q4, q5!

x_desired = -800 * 1e-3;
y_desired = 0 * 1e-3;
z_desired = 800* 1e-3;

duration = 10;
%tj = [a * duration**5 + b * duration **4 + ];
%dtj = [5 * a * time **4 ];

T = myobj.T_mat(duration);
%T = inv(T);

Qx = myobj.Q_mat(xs, x_desired);
C_mat_x = inv(T) * Qx;

Qy = myobj.Q_mat(ys, y_desired);
C_mat_y = inv(T)* Qy;

Qz = myobj.Q_mat(zs, z_desired);
C_mat_z = inv(T) * Qz;

s_intervals = 0.001;


list_x = [];
for i = 0: 0.001: duration
    b =  myobj.GraphWithValues(C_mat_x, i);
    list_x = [list_x; b];
end

list_y = [];
for i = 0: 0.001: duration
    b =  myobj.GraphWithValues(C_mat_y, i);
    list_y = [list_y; b];
end

list_z = [];
for i = 0: 0.001: duration
    b =  myobj.GraphWithValues(C_mat_z, i);
    list_z = [list_z; b];
end

%B = diff(myormalFuncobj.N(C_mat_z), timeo);

list_coordinates = [];
for i = 1:length(list_z)
    list_coordinates = [list_coordinates; [list_x(i), list_y(i), list_z(i)]];
end

% Velocity and acceleration in x direction at different times
vel_x = [];
for time = 0.01: 0.001: duration
    vel_x= [vel_x; myobj.FirstDer(C_mat_x,time)];
end

acc_x = [];
for time = 0.01: 0.001: duration
    acc_x= [acc_x; myobj.SecondDer(C_mat_x,time)];
end

% Velocity and acceleration in y direction at different times
vel_y = [];
for time = 0.01: 0.001: duration
    vel_y= [vel_y; myobj.FirstDer(C_mat_y,time)];
end

acc_y = [];
for time = 0.01: 0.001: duration
    acc_y= [acc_y; myobj.SecondDer(C_mat_y,time)];
end

% Velocity and acceleration in z direction at different times
vel_z = [];
for time = 0.01: 0.001: duration
    vel_z= [vel_z; myobj.FirstDer(C_mat_z,time)];
end

acc_z = [];
for time = 0.01: 0.001: duration
    acc_z= [acc_z; myobj.SecondDer(C_mat_z,time)];
end


vel_coordinates = [];
for i = 1:length(vel_x)
    vel_coordinates = [vel_coordinates; [vel_x(i), vel_y(i), vel_z(i)]];
end

acc_coordinates = [];
for i = 1:length(acc_x)
    acc_coordinates = [acc_coordinates; [acc_x(i), acc_y(i), acc_z(i)]];
end



b = [];
v = [];
j_val = [];
% Finding the angles to go to the desired position

for i = 0:s_intervals:10
    J = Jacobian(q1, q2, q3, q4, q5);
    inv_J = pinv(J);

    j_val = [j_val; J];
    %inv_J = pinv(J);
    
    dX = [myobj.IncrementalValues(C_mat_x,i, i+s_intervals);myobj.IncrementalValues(C_mat_y,i, i+s_intervals);myobj.IncrementalValues(C_mat_z,i, i+s_intervals)];

    %v = [v; [vel_x(i),vel_y(i),vel_z(i)]];

    Qs = inv_J * dX;

    b = [b; Qs(1), Qs(2), Qs(3), Qs(4), Qs(5)];

    q1 = q1 + Qs(1);
    q2 = q2 + Qs(2);
    q3 = q3 + Qs(3);
    q4 = q4 + Qs(4);
    q5 = q5 + Qs(5); 

    TE = ForwardKinematics(q1, q2, q3, q4, q5);
    xs = TE(1, 4);
    ys = TE(2, 4);
    zs = TE(3, 4);

    positions= [positions; [xs, ys, zs]];
    commands = [commands; [q1, q2, q3, q4, q5]];
end






% Second point


x_desired = -800 * 1e-3;
y_desired = -400 * 1e-3;
z_desired = 800* 1e-3;

duration = 10;
%tj = [a * duration**5 + b * duration **4 + ];
%dtj = [5 * a * time **4 ];

T = myobj.T_mat(duration);
%T = inv(T);

Qx = myobj.Q_mat(xs, x_desired);
C_mat_x = inv(T) * Qx;

Qy = myobj.Q_mat(ys, y_desired);
C_mat_y = inv(T)* Qy;

Qz = myobj.Q_mat(zs, z_desired);
C_mat_z = inv(T) * Qz;

s_intervals = 0.001;


list_x = [];
for i = 0: 0.001: duration
    b =  myobj.GraphWithValues(C_mat_x, i);
    list_x = [list_x; b];
end

list_y = [];
for i = 0: 0.001: duration
    b =  myobj.GraphWithValues(C_mat_y, i);
    list_y = [list_y; b];
end

list_z = [];
for i = 0: 0.001: duration
    b =  myobj.GraphWithValues(C_mat_z, i);
    list_z = [list_z; b];
end

%B = diff(myormalFuncobj.N(C_mat_z), timeo);

list_coordinates = [];
for i = 1:length(list_z)
    list_coordinates = [list_coordinates; [list_x(i), list_y(i), list_z(i)]];
end

% Velocity and acceleration in x direction at different times
vel_x = [];
for time = 0.01: 0.001: duration
    vel_x= [vel_x; myobj.FirstDer(C_mat_x,time)];
end

acc_x = [];
for time = 0.01: 0.001: duration
    acc_x= [acc_x; myobj.SecondDer(C_mat_x,time)];
end

% Velocity and acceleration in y direction at different times
vel_y = [];
for time = 0.01: 0.001: duration
    vel_y= [vel_y; myobj.FirstDer(C_mat_y,time)];
end

acc_y = [];
for time = 0.01: 0.001: duration
    acc_y= [acc_y; myobj.SecondDer(C_mat_y,time)];
end

% Velocity and acceleration in z direction at different times
vel_z = [];
for time = 0.01: 0.001: duration
    vel_z= [vel_z; myobj.FirstDer(C_mat_z,time)];
end

acc_z = [];
for time = 0.01: 0.001: duration
    acc_z= [acc_z; myobj.SecondDer(C_mat_z,time)];
end


vel_coordinates = [];
for i = 1:length(vel_x)
    vel_coordinates = [vel_coordinates; [vel_x(i), vel_y(i), vel_z(i)]];
end

acc_coordinates = [];
for i = 1:length(acc_x)
    acc_coordinates = [acc_coordinates; [acc_x(i), acc_y(i), acc_z(i)]];
end



b = [];
v = [];
j_val = [];
% Finding the angles to go to the desired position

for i = 0:s_intervals:10
    J = Jacobian(q1, q2, q3, q4, q5);
    inv_J = pinv(J);

    j_val = [j_val; J];
    %inv_J = pinv(J);
    
    dX = [myobj.IncrementalValues(C_mat_x,i, i+s_intervals);myobj.IncrementalValues(C_mat_y,i, i+s_intervals);myobj.IncrementalValues(C_mat_z,i, i+s_intervals)];

    %v = [v; [vel_x(i),vel_y(i),vel_z(i)]];

    Qs = inv_J * dX;

    b = [b; Qs(1), Qs(2), Qs(3), Qs(4), Qs(5)];

    q1 = q1 + Qs(1);
    q2 = q2 + Qs(2);
    q3 = q3 + Qs(3);
    q4 = q4 + Qs(4);
    q5 = q5 + Qs(5); 

    TE = ForwardKinematics(q1, q2, q3, q4, q5);
    xs = TE(1, 4);
    ys = TE(2, 4);
    zs = TE(3, 4);

    positions= [positions; [xs, ys, zs]];
    commands = [commands; [q1, q2, q3, q4, q5]];
end











% Third point


x_desired = 800 * 1e-3;
y_desired = -400 * 1e-3;
z_desired = 800* 1e-3;

duration = 10;
%tj = [a * duration**5 + b * duration **4 + ];
%dtj = [5 * a * time **4 ];

T = myobj.T_mat(duration);
%T = inv(T);

Qx = myobj.Q_mat(xs, x_desired);
C_mat_x = inv(T) * Qx;

Qy = myobj.Q_mat(ys, y_desired);
C_mat_y = inv(T)* Qy;

Qz = myobj.Q_mat(zs, z_desired);
C_mat_z = inv(T) * Qz;

s_intervals = 0.001;


list_x = [];
for i = 0: 0.001: duration
    b =  myobj.GraphWithValues(C_mat_x, i);
    list_x = [list_x; b];
end

list_y = [];
for i = 0: 0.001: duration
    b =  myobj.GraphWithValues(C_mat_y, i);
    list_y = [list_y; b];
end

list_z = [];
for i = 0: 0.001: duration
    b =  myobj.GraphWithValues(C_mat_z, i);
    list_z = [list_z; b];
end

%B = diff(myormalFuncobj.N(C_mat_z), timeo);

list_coordinates = [];
for i = 1:length(list_z)
    list_coordinates = [list_coordinates; [list_x(i), list_y(i), list_z(i)]];
end

% Velocity and acceleration in x direction at different times
vel_x = [];
for time = 0.01: 0.001: duration
    vel_x= [vel_x; myobj.FirstDer(C_mat_x,time)];
end

acc_x = [];
for time = 0.01: 0.001: duration
    acc_x= [acc_x; myobj.SecondDer(C_mat_x,time)];
end

% Velocity and acceleration in y direction at different times
vel_y = [];
for time = 0.01: 0.001: duration
    vel_y= [vel_y; myobj.FirstDer(C_mat_y,time)];
end

acc_y = [];
for time = 0.01: 0.001: duration
    acc_y= [acc_y; myobj.SecondDer(C_mat_y,time)];
end

% Velocity and acceleration in z direction at different times
vel_z = [];
for time = 0.01: 0.001: duration
    vel_z= [vel_z; myobj.FirstDer(C_mat_z,time)];
end

acc_z = [];
for time = 0.01: 0.001: duration
    acc_z= [acc_z; myobj.SecondDer(C_mat_z,time)];
end


vel_coordinates = [];
for i = 1:length(vel_x)
    vel_coordinates = [vel_coordinates; [vel_x(i), vel_y(i), vel_z(i)]];
end

acc_coordinates = [];
for i = 1:length(acc_x)
    acc_coordinates = [acc_coordinates; [acc_x(i), acc_y(i), acc_z(i)]];
end



b = [];
v = [];
j_val = [];
% Finding the angles to go to the desired position

for i = 0:s_intervals:10
    J = Jacobian(q1, q2, q3, q4, q5);
    inv_J = pinv(J);

    j_val = [j_val; J];
    %inv_J = pinv(J);
    
    dX = [myobj.IncrementalValues(C_mat_x,i, i+s_intervals);myobj.IncrementalValues(C_mat_y,i, i+s_intervals);myobj.IncrementalValues(C_mat_z,i, i+s_intervals)];

    %v = [v; [vel_x(i),vel_y(i),vel_z(i)]];

    Qs = inv_J * dX;

    b = [b; Qs(1), Qs(2), Qs(3), Qs(4), Qs(5)];

    q1 = q1 + Qs(1);
    q2 = q2 + Qs(2);
    q3 = q3 + Qs(3);
    q4 = q4 + Qs(4);
    q5 = q5 + Qs(5); 

    TE = ForwardKinematics(q1, q2, q3, q4, q5);
    xs = TE(1, 4);
    ys = TE(2, 4);
    zs = TE(3, 4);

    positions= [positions; [xs, ys, zs]];
    commands = [commands; [q1, q2, q3, q4, q5]];
end















% Fourth point


x_desired = 800 * 1e-3;
y_desired = 400 * 1e-3;
z_desired = 800* 1e-3;

duration = 10;
%tj = [a * duration**5 + b * duration **4 + ];
%dtj = [5 * a * time **4 ];

T = myobj.T_mat(duration);
%T = inv(T);

Qx = myobj.Q_mat(xs, x_desired);
C_mat_x = inv(T) * Qx;

Qy = myobj.Q_mat(ys, y_desired);
C_mat_y = inv(T)* Qy;

Qz = myobj.Q_mat(zs, z_desired);
C_mat_z = inv(T) * Qz;

s_intervals = 0.001;


list_x = [];
for i = 0: 0.001: duration
    b =  myobj.GraphWithValues(C_mat_x, i);
    list_x = [list_x; b];
end

list_y = [];
for i = 0: 0.001: duration
    b =  myobj.GraphWithValues(C_mat_y, i);
    list_y = [list_y; b];
end

list_z = [];
for i = 0: 0.001: duration
    b =  myobj.GraphWithValues(C_mat_z, i);
    list_z = [list_z; b];
end

%B = diff(myormalFuncobj.N(C_mat_z), timeo);

list_coordinates = [];
for i = 1:length(list_z)
    list_coordinates = [list_coordinates; [list_x(i), list_y(i), list_z(i)]];
end

% Velocity and acceleration in x direction at different times
vel_x = [];
for time = 0.01: 0.001: duration
    vel_x= [vel_x; myobj.FirstDer(C_mat_x,time)];
end

acc_x = [];
for time = 0.01: 0.001: duration
    acc_x= [acc_x; myobj.SecondDer(C_mat_x,time)];
end

% Velocity and acceleration in y direction at different times
vel_y = [];
for time = 0.01: 0.001: duration
    vel_y= [vel_y; myobj.FirstDer(C_mat_y,time)];
end

acc_y = [];
for time = 0.01: 0.001: duration
    acc_y= [acc_y; myobj.SecondDer(C_mat_y,time)];
end

% Velocity and acceleration in z direction at different times
vel_z = [];
for time = 0.01: 0.001: duration
    vel_z= [vel_z; myobj.FirstDer(C_mat_z,time)];
end

acc_z = [];
for time = 0.01: 0.001: duration
    acc_z= [acc_z; myobj.SecondDer(C_mat_z,time)];
end


vel_coordinates = [];
for i = 1:length(vel_x)
    vel_coordinates = [vel_coordinates; [vel_x(i), vel_y(i), vel_z(i)]];
end

acc_coordinates = [];
for i = 1:length(acc_x)
    acc_coordinates = [acc_coordinates; [acc_x(i), acc_y(i), acc_z(i)]];
end



b = [];
v = [];
j_val = [];
% Finding the angles to go to the desired position

for i = 0:s_intervals:10
    J = Jacobian(q1, q2, q3, q4, q5);
    inv_J = pinv(J);

    j_val = [j_val; J];
    %inv_J = pinv(J);
    
    dX = [myobj.IncrementalValues(C_mat_x,i, i+s_intervals);myobj.IncrementalValues(C_mat_y,i, i+s_intervals);myobj.IncrementalValues(C_mat_z,i, i+s_intervals)];

    %v = [v; [vel_x(i),vel_y(i),vel_z(i)]];

    Qs = inv_J * dX;

    b = [b; Qs(1), Qs(2), Qs(3), Qs(4), Qs(5)];

    q1 = q1 + Qs(1);
    q2 = q2 + Qs(2);
    q3 = q3 + Qs(3);
    q4 = q4 + Qs(4);
    q5 = q5 + Qs(5); 

    TE = ForwardKinematics(q1, q2, q3, q4, q5);
    xs = TE(1, 4);
    ys = TE(2, 4);
    zs = TE(3, 4);

    positions= [positions; [xs, ys, zs]];
    commands = [commands; [q1, q2, q3, q4, q5]];
end






% Fifth point


x_desired = -800 * 1e-3;
y_desired = 400 * 1e-3;
z_desired = 800* 1e-3;

duration = 10;
%tj = [a * duration**5 + b * duration **4 + ];
%dtj = [5 * a * time **4 ];

T = myobj.T_mat(duration);
%T = inv(T);

Qx = myobj.Q_mat(xs, x_desired);
C_mat_x = inv(T) * Qx;

Qy = myobj.Q_mat(ys, y_desired);
C_mat_y = inv(T)* Qy;

Qz = myobj.Q_mat(zs, z_desired);
C_mat_z = inv(T) * Qz;

s_intervals = 0.001;


list_x = [];
for i = 0: 0.001: duration
    b =  myobj.GraphWithValues(C_mat_x, i);
    list_x = [list_x; b];
end

list_y = [];
for i = 0: 0.001: duration
    b =  myobj.GraphWithValues(C_mat_y, i);
    list_y = [list_y; b];
end

list_z = [];
for i = 0: 0.001: duration
    b =  myobj.GraphWithValues(C_mat_z, i);
    list_z = [list_z; b];
end

%B = diff(myormalFuncobj.N(C_mat_z), timeo);

list_coordinates = [];
for i = 1:length(list_z)
    list_coordinates = [list_coordinates; [list_x(i), list_y(i), list_z(i)]];
end

% Velocity and acceleration in x direction at different times
vel_x = [];
for time = 0.01: 0.001: duration
    vel_x= [vel_x; myobj.FirstDer(C_mat_x,time)];
end

acc_x = [];
for time = 0.01: 0.001: duration
    acc_x= [acc_x; myobj.SecondDer(C_mat_x,time)];
end

% Velocity and acceleration in y direction at different times
vel_y = [];
for time = 0.01: 0.001: duration
    vel_y= [vel_y; myobj.FirstDer(C_mat_y,time)];
end

acc_y = [];
for time = 0.01: 0.001: duration
    acc_y= [acc_y; myobj.SecondDer(C_mat_y,time)];
end

% Velocity and acceleration in z direction at different times
vel_z = [];
for time = 0.01: 0.001: duration
    vel_z= [vel_z; myobj.FirstDer(C_mat_z,time)];
end

acc_z = [];
for time = 0.01: 0.001: duration
    acc_z= [acc_z; myobj.SecondDer(C_mat_z,time)];
end


vel_coordinates = [];
for i = 1:length(vel_x)
    vel_coordinates = [vel_coordinates; [vel_x(i), vel_y(i), vel_z(i)]];
end

acc_coordinates = [];
for i = 1:length(acc_x)
    acc_coordinates = [acc_coordinates; [acc_x(i), acc_y(i), acc_z(i)]];
end



b = [];
v = [];
j_val = [];
% Finding the angles to go to the desired position

for i = 0:s_intervals:10
    J = Jacobian(q1, q2, q3, q4, q5);
    inv_J = pinv(J);

    j_val = [j_val; J];
    %inv_J = pinv(J);
    
    dX = [myobj.IncrementalValues(C_mat_x,i, i+s_intervals);myobj.IncrementalValues(C_mat_y,i, i+s_intervals);myobj.IncrementalValues(C_mat_z,i, i+s_intervals)];

    %v = [v; [vel_x(i),vel_y(i),vel_z(i)]];

    Qs = inv_J * dX;

    b = [b; Qs(1), Qs(2), Qs(3), Qs(4), Qs(5)];

    q1 = q1 + Qs(1);
    q2 = q2 + Qs(2);
    q3 = q3 + Qs(3);
    q4 = q4 + Qs(4);
    q5 = q5 + Qs(5); 

    TE = ForwardKinematics(q1, q2, q3, q4, q5);
    xs = TE(1, 4);
    ys = TE(2, 4);
    zs = TE(3, 4);

    positions= [positions; [xs, ys, zs]];
    commands = [commands; [q1, q2, q3, q4, q5]];
end




% Final point


x_desired = -650.8 * 1e-3;
y_desired = 0 * 1e-3;
z_desired = 758.8* 1e-3;

duration = 10;
%tj = [a * duration**5 + b * duration **4 + ];
%dtj = [5 * a * time **4 ];

T = myobj.T_mat(duration);
%T = inv(T);

Qx = myobj.Q_mat(xs, x_desired);
C_mat_x = inv(T) * Qx;

Qy = myobj.Q_mat(ys, y_desired);
C_mat_y = inv(T)* Qy;

Qz = myobj.Q_mat(zs, z_desired);
C_mat_z = inv(T) * Qz;

s_intervals = 0.001;


list_x = [];
for i = 0: 0.001: duration
    b =  myobj.GraphWithValues(C_mat_x, i);
    list_x = [list_x; b];
end

list_y = [];
for i = 0: 0.001: duration
    b =  myobj.GraphWithValues(C_mat_y, i);
    list_y = [list_y; b];
end

list_z = [];
for i = 0: 0.001: duration
    b =  myobj.GraphWithValues(C_mat_z, i);
    list_z = [list_z; b];
end

%B = diff(myormalFuncobj.N(C_mat_z), timeo);

list_coordinates = [];
for i = 1:length(list_z)
    list_coordinates = [list_coordinates; [list_x(i), list_y(i), list_z(i)]];
end

% Velocity and acceleration in x direction at different times
vel_x = [];
for time = 0.01: 0.001: duration
    vel_x= [vel_x; myobj.FirstDer(C_mat_x,time)];
end

acc_x = [];
for time = 0.01: 0.001: duration
    acc_x= [acc_x; myobj.SecondDer(C_mat_x,time)];
end

% Velocity and acceleration in y direction at different times
vel_y = [];
for time = 0.01: 0.001: duration
    vel_y= [vel_y; myobj.FirstDer(C_mat_y,time)];
end

acc_y = [];
for time = 0.01: 0.001: duration
    acc_y= [acc_y; myobj.SecondDer(C_mat_y,time)];
end

% Velocity and acceleration in z direction at different times
vel_z = [];
for time = 0.01: 0.001: duration
    vel_z= [vel_z; myobj.FirstDer(C_mat_z,time)];
end

acc_z = [];
for time = 0.01: 0.001: duration
    acc_z= [acc_z; myobj.SecondDer(C_mat_z,time)];
end


vel_coordinates = [];
for i = 1:length(vel_x)
    vel_coordinates = [vel_coordinates; [vel_x(i), vel_y(i), vel_z(i)]];
end

acc_coordinates = [];
for i = 1:length(acc_x)
    acc_coordinates = [acc_coordinates; [acc_x(i), acc_y(i), acc_z(i)]];
end



b = [];
v = [];
j_val = [];
% Finding the angles to go to the desired position

for i = 0:s_intervals:10
    J = Jacobian(q1, q2, q3, q4, q5);
    inv_J = pinv(J);

    j_val = [j_val; J];
    %inv_J = pinv(J);
    
    dX = [myobj.IncrementalValues(C_mat_x,i, i+s_intervals);myobj.IncrementalValues(C_mat_y,i, i+s_intervals);myobj.IncrementalValues(C_mat_z,i, i+s_intervals)];

    %v = [v; [vel_x(i),vel_y(i),vel_z(i)]];

    Qs = inv_J * dX;

    b = [b; Qs(1), Qs(2), Qs(3), Qs(4), Qs(5)];

    q1 = q1 + Qs(1);
    q2 = q2 + Qs(2);
    q3 = q3 + Qs(3);
    q4 = q4 + Qs(4);
    q5 = q5 + Qs(5); 

    TE = ForwardKinematics(q1, q2, q3, q4, q5);
    xs = TE(1, 4);
    ys = TE(2, 4);
    zs = TE(3, 4);

    positions= [positions; [xs, ys, zs]];
    commands = [commands; [q1, q2, q3, q4, q5]];
end








commands = rad2deg(commands);

offsets = [0, 210, 210, 0, 120];
out_commands = offsets + commands;
out_commands = out_commands; %%rem(out_commands,360); %% 360;

%{
l = [];
for i = 2:length(out_commands)
    difference = abs(out_commands(i-1) - out_commands(i));
    if difference > 10
        l = [l; out_commands(i-1), out_commands(i)];
    end
end

k = myobj.GraphWithValues(C_mat_x,10-0.01);
h = myobj.GraphWithValues(C_mat_x,10);
f = myobj.IncrementalValues(C_mat_x,10-0.01,10);


d = [];
for i = 0:0.01:10
    d = [d; myobj.IncrementalValues(C_mat_x,i, i+0.01), myobj.IncrementalValues(C_mat_y,i, i+0.01), myobj.IncrementalValues(C_mat_z,i, i+0.01)];
end

%}
csvwrite("new_moving_coordinates.csv", out_commands)

