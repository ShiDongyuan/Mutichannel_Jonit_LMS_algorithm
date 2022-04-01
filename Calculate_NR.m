%% Clean memory and workspace 
clc      ;
close all; 
clear    ;

%% Loading the mat file from worksapce 
load('Noise_generation.mat', 'Distur_matrix');
d1_dBA = 20*log10(rms(Distur_matrix(:,1)));
d2_dBA = 20*log10(rms(Distur_matrix(:,2)));
d3_dBA = 20*log10(rms(Distur_matrix(:,3)));
d4_dBA = 20*log10(rms(Distur_matrix(:,4)));

Dir_dBA = [d1_dBA; d2_dBA; d3_dBA; d4_dBA];

a = load('Four_channel_ANC_tst_Program.mat');
b = load('Tst_4channel_program_McFxLMS.mat');

Joint_e1_dBA = 20*log10(rms(a.Err_v(end-4096:end,1)));
Joint_e2_dBA = 20*log10(rms(a.Err_v(end-4096:end,2)));
Joint_e3_dBA = 20*log10(rms(a.Err_v(end-4096:end,3)));
Joint_e4_dBA = 20*log10(rms(a.Err_v(end-4096:end,4)));

Joint_Er_dBA = [Joint_e1_dBA; Joint_e2_dBA; Joint_e3_dBA; Joint_e4_dBA];

FxLMS_e1_dBA = 20*log10(rms(b.Err_array(end-4096:end,1)));
FxLMS_e2_dBA = 20*log10(rms(b.Err_array(end-4096:end,2)));
FxLMS_e3_dBA = 20*log10(rms(b.Err_array(end-4096:end,3)));
FxLMS_e4_dBA = 20*log10(rms(b.Err_array(end-4096:end,4)));

FxLMS_Er_dBA = [FxLMS_e1_dBA; FxLMS_e2_dBA; FxLMS_e3_dBA; FxLMS_e4_dBA];

Joint_NR = Dir_dBA - Joint_Er_dBA ;
FxLMS_NR = Dir_dBA - FxLMS_Er_dBA ;

set(groot,'defaultAxesTickLabelInterpreter','latex')
X = categorical({'Error 1','Error 2','Error 3','Error 4'});
X = reordercats(X,{'Error 1','Error 2','Error 3','Error 4'});
bar_value = [FxLMS_NR, Joint_NR];
bar(X,bar_value)
legend({'McFxLMS','Multichannel joint LMS'},'Interpreter','latex');
ylabel('Noise reduction (dB)','Interpreter','latex');
title('Noise reduction level at different error microphones','Interpreter','latex');
grid on 