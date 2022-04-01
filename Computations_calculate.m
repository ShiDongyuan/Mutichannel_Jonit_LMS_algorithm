%% Clean memory and worksapce 
clear    ;
close all; 
clc      ;
%% System configuration 
Len_c = 512 ;
Len_s = 256 ;
Num_r = 1   ;
Num_s = 1   ;
Num_e = 1   ;

[McFxLMS_additions, McFxLMS_multiplicaitons] = full_connection_FxLMS(Len_c, Len_s, Num_r, Num_s, Num_e);
[Joint_additions, Joint_multiplicaitons]     = computations_Multichannel_jointLMS(Len_c, Len_s, Num_r, Num_s, Num_e);

Num = 10 ;
Addition_vector = zeros(Num,2);
Multipli_vector = zeros(Num,2);
for Chann_num = 1:Num
    Num_r = Chann_num ;
    Num_s = Chann_num ;
    Num_e = Chann_num ;
    [Addition_vector(Chann_num,1),Multipli_vector(Chann_num,1)] = full_connection_FxLMS(Len_c, Len_s, Num_r, Num_s, Num_e);
    [Addition_vector(Chann_num,2),Multipli_vector(Chann_num,2)] = computations_Multichannel_jointLMS(Len_c, Len_s, Num_r, Num_s, Num_e);
end
set(groot,'defaultAxesTickLabelInterpreter','latex')
figure ;
subplot(1,2,1);
plot(1:Num,Addition_vector(:,1),'-^',1:Num,Addition_vector(:,2),'-^');
axis([1,inf,-inf, inf])
grid on ;
xlabel('Number of channels','Interpreter','latex');
ylabel('Additions','Interpreter','latex');
legend({'McFxLMS','Multichannel Joint LMS'},'Interpreter','latex');

subplot(1,2,2);
plot(1:Num,Multipli_vector(:,1),'-s',1:Num,Multipli_vector(:,2),'-s');
grid on ;
axis([1,inf,-inf, inf])
ylabel('Multiplications','Interpreter','latex')
xlabel('Number of channels','Interpreter','latex');
legend({'McFxLMS','Multichannel Joint LMS'},'Interpreter','latex');

%% computing the computations of the McFxLMS algorithm 
function [additions, multiplicaitons] = full_connection_FxLMS(Len_c, Len_s, Num_r, Num_s, Num_e)
    Channel_num                                                = Num_r * Num_s                                         ;
    [single_channel_additions, single_channel_multiplicaitons] = computations_of_one_FxLMS_Channel(Len_c, Len_s, Num_e);
    additions                                                  = Channel_num * single_channel_additions                ;
    multiplicaitons                                            = Channel_num * single_channel_multiplicaitons          ;
end 
function [additions, multiplicaitons] = computations_of_one_FxLMS_Channel(Len_c, Len_s, Num_e)
    % Filtering signal computations 
    % X'1 = x0*s0 + x1*s1 + ......
    addition_num_1       = (Len_s-1)*Num_e;
    multiplicaiton_num_1 =  Len_s*Num_e   ;
    
    % Weight updating 
    % Wc = Wc + u * (e1*X'1 + e2*X'2+ ......)
    addition_num_2       = Len_c + Len_c*(Num_e-1) ;
    multiplicaiton_num_2 = Num_e + Len_c*Num_e ;
    
    additions       = addition_num_1       + addition_num_2      ;
    multiplicaitons = multiplicaiton_num_1 + multiplicaiton_num_2;
end
%% computing the computations of the Multichannel Joint LMS algorithm 
function [additions, multiplicaitons] = computations_Multichannel_jointLMS(Len_c, Len_s, Num_r, Num_s, Num_e)
    % Filter err signal 
    % e1' = e1(n)*s_L + e1(n-1)*s_L-1 + ....
    addition_num_1       = (Len_s-1)*Num_e; 
    multiplication_num_1 = Len_s*Num_e    ;
    
    % Weight updating computaitons 
    % Wc = Wc + u *(e1' + e2' + ....)X1
    addition_num_2       = Num_r*(Len_c +(Num_e-1)) ;
    multiplication_num_2 = Num_r*Len_c +1           ;
    
    % Total computation 
    additions       = Num_s*(addition_num_1+addition_num_2);
    multiplicaitons = Num_s*(multiplication_num_1+multiplication_num_2);
end

