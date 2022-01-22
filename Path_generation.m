%% Clean memory and workspace
clc       ;
close all ;
clear     ;

%% Generating the primary path
fs      = 16000 ;% system sampling rate
Fc_low  = 200   ;% low cut-off frequency
Fc_high = 5000  ;% High cut-off frequnecy
Wc_low  = Fc_low*2/fs  ;
Wc_high = Fc_high*2/fs ;

Len = 256 ;
Primary_path_matrix = zeros(4,Len,4);
for Refer_item = 1:4
    for Err_item = 1:4
        Primary_path = awgn(fir1(Len-1,[Wc_low Wc_high]),50);
        Primary_path_matrix(Err_item,:,Refer_item) = Primary_path' ;
    end
end
%Primary_path = awgn(fir1(Len-1,[Wc_low Wc_high]),50);
%freqz(Primary_path_matrix(2,:,3),1,512);

Ls = 64 ;
Secondary_path_matrix = zeros(4,Ls,4);
for Secon_item = 1:4
    for Err_item = 1:4
        Secondary_path = awgn(fir1(Ls-1,[Wc_low Wc_high]),50);
        Secondary_path_matrix(Err_item,:,Secon_item) = Secondary_path';
    end
end
freqz(Secondary_path_matrix(2,:,3),1,512);
save('Path_generation.mat','Primary_path_matrix','Secondary_path_matrix');
