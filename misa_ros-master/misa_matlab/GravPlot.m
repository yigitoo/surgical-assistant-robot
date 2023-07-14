
close all
clear all
clc

iter = 0.01;
q1 = 0;
q2 = 0;
q3 = 0;

M1 = [];
M2 = [];
M3 = [];
M4 = [];
X = [];
endeffector_mass = 0.350;

for q = 0 : iter : 2*pi
    q1 = q;
    q2 = q;
    P = CalcGrav_v2_test(q1,q2,q3,endeffector_mass);
    M1 = [P(1), M1];
    M2 = [P(2), M2];
    M3 = [P(3), M3];
    M4 = [P(4), M4];
    X = [q, X];
end
hold on
grid on
plot(X,M1,'LineWidth', 2);
plot(X,M2, 'LineWidth', 2);
plot(X,M3, 'LineWidth', 2);
plot(X,M4, 'LineWidth', 2);

title("Numeric Gravity Compensation values");
ylabel("Torque (N/m)");
xlabel("Angle (rad)");

legend("Joint 1", "Joint 2", "Joint 3", "Joint 4");

hold off
