%% Clean memory and workspace
clc       ;
close all ;
clear     ;

%% Loading primary noise and disturbances
load('Noise_generation.mat');

%% Loading the primary and secondary paths
load('Path_generation.mat');

%% Configuring the element of the joint LMS algorithm
Len = 512*2 ;
Ls  = size(Secondary_path_matrix(1,:,1),2);

%% Shorten the input data 
raw_len       = size(Distur_matrix,1)       ;
new_len       = floor(raw_len/2)            ;
Refer_matrix  = Refer_matrix(1:new_len,:,:) ;
Distur_matrix = Distur_matrix(1:new_len,:,:);

%% Calculating the theoritical step size bound 
[step_size, Step_size_vector] = Step_size_bound_calculation(Refer_matrix, 4, Len, Ls);
disp("The calculated step size bound is : "+ num2str(step_size));

%% Multichannel Joint LMS algorithm 
muw     = step_size ;
a       = 0.1*muw   ;
b       = 10*muw    ;
delta   = 0.01       ;
f =@Tst_step_size_multichannel_joint ;
Lambda = Godden_section_search(f,a,b,delta);
disp("Lambda is : "+Lambda);
disp("Function value is :"+f(Lambda));
%f_value = Tst_step_size_multichannel_joint(muw);% Len, Ls, Secondary_path_matrix, Distur_matrix, Refer_matrix

%% Function of multichannel LMS ALGORITHM
function f_value = Tst_step_size_multichannel_joint(muw)
    %% Counting the number 
    persistent Num ;
    if isempty(Num)
        Num = 0 ;
    end
    
    %% Loading primary noise and disturbances
    load('Noise_generation.mat');
    
    %% Loading the primary and secondary paths
    load('Path_generation.mat');

    %% Configuring the element of the joint LMS algorithm
    Len = 512*2 ;
    Ls  = size(Secondary_path_matrix(1,:,1),2);

    %% Shorten the input data 
    raw_len       = size(Distur_matrix,1)       ;
    new_len       = floor(raw_len/6)            ;
    Refer_matrix  = Refer_matrix(1:new_len,:,:) ;
    Distur_matrix = Distur_matrix(1:new_len,:,:);

    for Secon_item = 1:4
            Joint_matrix(Secon_item) = FourChan_Joint_LMS_Element_c(Len,Ls, Secondary_path_matrix(1,:,Secon_item)',...
            Secondary_path_matrix(2,:,Secon_item)',Secondary_path_matrix(3,:,Secon_item)',Secondary_path_matrix(4,:,Secon_item)');
    end
    
    %% Filtering
%muw   =  2.3*1.085822123055911e-05; %0.000013 ;
    N     = size(Distur_matrix,1);
    Err_v = zeros(N,4)           ;
    Er    = zeros(4,1)           ;
    Wc_matrix = zeros(Len,4,4)   ;

    for ii = 1:N
        Yt = zeros(4,1);
        for Secon_item = 1:4
            [yt_1, yt_2, yt_3, yt_4, Joint_matrix(Secon_item)] = Joint_matrix(Secon_item).controller(muw, Refer_matrix(ii,1),...
             Refer_matrix(ii,2), Refer_matrix(ii,3), Refer_matrix(ii,4),...
              Er(1,1), Er(2,1), Er(3,1), Er(4,1));
            Yt(1,1) = Yt(1,1) + yt_1 ;
            Yt(2,1) = Yt(2,1) + yt_2 ;
            Yt(3,1) = Yt(3,1) + yt_3 ;
            Yt(4,1) = Yt(4,1) + yt_4 ;
        end
        Er = Distur_matrix(ii,:)' - Yt ;
        Err_v(ii,:) = Er' ;
    end

    for Secon_item = 1:4
        Wc_matrix(:,1,Secon_item) = Joint_matrix(Secon_item).Wc_1 ;
        Wc_matrix(:,2,Secon_item) = Joint_matrix(Secon_item).Wc_2 ;
        Wc_matrix(:,3,Secon_item) = Joint_matrix(Secon_item).Wc_3 ;
        Wc_matrix(:,4,Secon_item) = Joint_matrix(Secon_item).Wc_4 ;
    end
    
    err_star = Err_v(1:1024,4);
    err_end  = Err_v(end-1023:end,4);
    f_value  = Fitenss_LMS(err_star,err_end);
    
    %% Drawing figure
    figure          ;
    plot(Err_v(:,4));
    grid on         ;
    
    %% Display the approximate convergence speed 
    Num = Num + 1 ;
    disp("<<======================("+num2str(Num)+")======================>>")
    disp("The approximate convergence speed : "+ num2str(f_value));
end