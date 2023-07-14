close all
clear
clc

% Setup
% Kinova/2 + Pipe + Elbow + Pipe + Kinova/2
% Vectors
% x + x + xz + z + z

%Lengths in meters.

CKinova = 0;
CLPipe = 0.04425;
CLElbow = 0.005;

PPipe_1 = [CLPipe, 0,0];

PElbow_h = PPipe_1 + [CLPipe + CLElbow, 0, 0];

PElbow_v = PElbow_h + [CLElbow, 0, CLElbow];

PPipe_2 = PElbow_v + [0, 0, CLElbow + CLPipe];

PKinova_2 = PPipe_2 + [0, 0, CLPipe];

% m1 = M1Rot(0.061kg) + Elbow1(0.121kg) + M2Fix(0.061kg) + Kinova/2(0.1785kg)
% m2 = M2Rot(0.061kg) + Elbow2(0.121kg) + M3Fix(0.061kg) + Kinova(0.357kg)
% m3 = M3Rot(0.061kg) + Elbow3(0.121kg) + M4Fix(0.061kg) + Kinova(0.357kg)
% m4 = M4Rot(0.037kg) + Kinova/2(0.1785kg)

Masses = transpose([0.061, 0.061, 0.061, 0.061, 2 * 0.17585]);
Points = transpose([PPipe_1 ; PElbow_h; PElbow_v; PPipe_2; PKinova_2]);

[center_of_mass,mass] = centerFinder(Points,Masses);

