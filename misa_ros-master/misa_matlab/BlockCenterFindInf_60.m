close all
clear
clc
% CONFIGURATION
% --------------
% Kinova/2 0.174 grams
% Pipe - Motor screws
% Vertical Pipe (Secondary axis) 132 grams
% Pipe screws
% Elbow screws - 60 grams
% Elbow - 184.22 grams
% Elbow screws
% Horizontal screws - 20 grams
% Horizontal Pipe (Primary axis) 132 grams
% Kinova / 2
% Motor - Pipe screws
% Dummy motor distance gap

% Vectors of objects from beginning to end:
% masses:
mass_kinova = 0.348;
mass_elbow = 0.184; % for 100% infill ratio
mass_pipe = 0.132 * 0.6;
mass_elbow_screws = 0.060; % for 6 screws
mass_motor_screws = 0.020; % for 6 screws
mass_flange_screws = 0.015;

total_mass = mass_kinova/2 ...
             + mass_motor_screws ...
             + mass_pipe ...
             + mass_flange_screws ...
             + mass_elbow_screws ...
             + mass_elbow ...
             + mass_elbow_screws ...
             + mass_flange_screws ...
             + mass_pipe ...
             + mass_motor_screws ...
             + mass_kinova / 2 ;

         
compoint_f_kinova = [0; 0; 0];
compoint_f_motor_screws = [0.001; 0; 0]; 
compoint_f_pipe = compoint_f_motor_screws + [0.061/2; 0; 0];
compoint_f_elbow_screws = compoint_f_pipe + [0.061/2; 0; 0] - [0.011; 0; 0];
compoint_elbow = compoint_f_elbow_screws + [0.011; 0; 0] + [0.119221 / 2; 0; 0.119221/2];
compoint_s_elbow_screws = compoint_elbow + [0.119221/2; 0; 0.119221/2 ] - [0; 0; 0.011];
compoint_s_pipe = compoint_s_elbow_screws + [0; 0; 0.061/2];
compoint_s_motor_screws = compoint_s_pipe + [0;0 ; 0.061/2] - [0; 0; 0.0035]; 
compoint_s_kinova = compoint_s_motor_screws + [0; 0; 0.035] + [0 ; 0; 0.001];

compoint_unnorm    =   compoint_f_kinova * mass_kinova / 2 ...
                   + compoint_f_motor_screws * mass_motor_screws ...
                   + compoint_f_pipe * mass_pipe ...
                   + compoint_f_elbow_screws * (mass_flange_screws + mass_elbow_screws) ...
                   + compoint_elbow * mass_elbow ...
                   + compoint_s_elbow_screws * (mass_flange_screws + mass_elbow_screws) ...
                   + compoint_s_pipe * mass_pipe ...
                   + compoint_s_motor_screws * mass_motor_screws ...
                   + compoint_s_kinova * mass_kinova / 2 ;
               
compoint_general = compoint_unnorm ./ total_mass ;
end_point = [0.0005 + 0.119221 + 0.061 ; 0.000001 ; 0.005 + 0.119221 + 0.061];

compoint_ratio_general = compoint_general ./ end_point
