%% Clean memory and workspace
clc       ;
close all ;
clear     ;

%% Loading primary noise and disturbances
load('Noise_generation.mat');

%% Loading the primary and secondary paths
load('Path_generation.mat');

%% Configuring the element of the joint LMS algorithm
Len = 512 ;
Ls  = size(Secondary_path_matrix(1,:,1),2);

%% Caculating the bound of the step size for multichannel joint LMS algorithm. 
Ref     = Refer_matrix ;
Len_c   = Len          ;
Err_Num = 4            ;
Len_s   = Ls           ;
[step_size, Step_size_vector] = Step_size_bound_calculation(Ref, Err_Num, Len_c, Len_s);

% seting the step sizes
muw_s = [0.01, 0.05, 0.1, 0.5, 1, 1.5]*step_size;