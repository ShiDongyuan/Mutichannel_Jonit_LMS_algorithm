%% Clean memory and workspace
clc       ;
close all ;
clear     ;

%% Loading the primary and secondary paths
load('Path_generation.mat')

%% System configuration
fs = 16000     ;
T  = 15         ;
t  = 0:1/fs:T  ;
N  = length(t) ;

%% Constructing the primary noise
Fc_low  = 400   ;% low cut-off frequency
Fc_high = 3200  ;% High cut-off frequnecy
Wc_low  = Fc_low*2/fs  ;
Wc_high = Fc_high*2/fs ;
FIR     = fir1(511,[Wc_low Wc_high]);

% Generating the primary noise
Refer_matrix = zeros(N,4);
for Refer_item = 1:4
    Reference = filter(FIR',1,randn(1,N));
    Refer_matrix(:,Refer_item) = Reference ;
end
% Generating the disturbance
Distur_matrix = zeros(N,4);
for Err_item = 1:4
    Disturbance = zeros(N,1);
    for Refer_item = 1:4
        Disturbance = Disturbance + filter(Primary_path_matrix(Err_item,:,Refer_item) ,1,Refer_matrix(:,Refer_item));
    end
    Distur_matrix(:,Err_item) = Disturbance ;
end

save('Noise_generation.mat','Refer_matrix','Distur_matrix');
