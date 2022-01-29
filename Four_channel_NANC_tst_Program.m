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
%Joint_matrix = {} ;
for Secon_item = 1:4
        Joint_matrix(Secon_item) = FourChan_Normalized_Joint_LMS_Element_c(Len,Ls, Secondary_path_matrix(1,:,Secon_item)',...
        Secondary_path_matrix(2,:,Secon_item)',Secondary_path_matrix(3,:,Secon_item)',Secondary_path_matrix(4,:,Secon_item)');
end

%% Filtering
muw   = 0.001;
N     = size(Distur_matrix,1);
Yd    = zeros(Ls,4,4)        ;
Err_v = zeros(N,4)           ;
Er = zeros(4,1);
Wc_matrix = zeros(Len,4,4)    ;

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
%% Drawing figure
figure          ;
plot(Err_v(:,4));
grid on         ;

figure
freqz(Wc_matrix(:,1,1)',1,512);
save('Four_channel_NANC_tst_Program.mat','Err_v','Wc_matrix');


