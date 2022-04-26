%% Clean memory and workspace
clc       ;
close all ;
clear     ;

%% Generating the distrubance and reference and loading the paths 
Fc_low  = 400  ;
Fc_high = 1200 ;
T       = 15/8    ;
global Refer_matrix Distur_matrix Primary_path_matrix Secondary_path_matrix;
[Refer_matrix, Distur_matrix,Primary_path_matrix,Secondary_path_matrix] = Noise_generation_function(Fc_low, Fc_high, T);

%% Configuring the element of the joint LMS algorithm
global Len Ls ;
Len = 512     ;
Ls  = size(Secondary_path_matrix(1,:,1),2);

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


%% Function of multichannel LMS ALGORITHM
function f_value = Tst_step_size_multichannel_joint(muw)

    global Refer_matrix Distur_matrix Primary_path_matrix Secondary_path_matrix;
    global Len Ls;
    %% Counting the number 
    persistent Num ;
    if isempty(Num)
        Num = 0 ;
    end
    
    for Secon_item = 1:4
            Joint_matrix(Secon_item) = FourChan_Joint_LMS_Element_c(Len,Ls, Secondary_path_matrix(1,:,Secon_item)',...
            Secondary_path_matrix(2,:,Secon_item)',Secondary_path_matrix(3,:,Secon_item)',Secondary_path_matrix(4,:,Secon_item)');
    end
	
	%% Filtering
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