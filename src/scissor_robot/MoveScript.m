clear;
clc;

q1 = 0; q2 = 0; q3 = 0; q4 = 0; q5 = deg2rad(90);

TE = ForwardKinematics(q1, q2, q3, q4, q5);
xs = TE(1, 4);
ys = TE(2, 4);
zs = TE(3, 4);

% Sub task 1: Increase x 
commands = [];
positions = [];

d = 150*1e-3;
commands = [commands; [q1, q2, q3, q4, q5]];
positions= [positions; [xs, ys, zs]];

% Go d/2 on + y direction.
for di = 0 : 0.0001: d/2

    J = Jacobian(q1, q2, q3, q4, q5);
    inv_J = pinv(J);
    % Q = inv(J) * dX;
    dX = [0; 0.0001; 0];
    Qs = inv_J * dX;
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
% Go d/2 on -z direction
for di = 0 : 0.0001: d/2

    J = Jacobian(q1, q2, q3, q4, q5);
    inv_J = pinv(J);
    % Q = inv(J) * dX;
    dX = [0; 0; -0.0001];
    Qs = inv_J * dX;
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
% Go d on - y direction.
for di = 0 : 0.0001: d

    J = Jacobian(q1, q2, q3, q4, q5);
    inv_J = pinv(J);
    % Q = inv(J) * dX;
    dX = [0; -0.0001; 0];
    Qs = inv_J * dX;
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
% Go d/2 on +z direction.
for di = 0 : 0.0001: d/2

    J = Jacobian(q1, q2, q3, q4, q5);
    inv_J = pinv(J);
    % Q = inv(J) * dX;
    dX = [0; 0; 0.0001];
    Qs = inv_J * dX;
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
% Go d/2 on +y direction.
for di = 0 : 0.0001: d/2

    J = Jacobian(q1, q2, q3, q4, q5);
    inv_J = pinv(J);
    % Q = inv(J) * dX;
    dX = [0; 0.0001; 0];
    Qs = inv_J * dX;
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

