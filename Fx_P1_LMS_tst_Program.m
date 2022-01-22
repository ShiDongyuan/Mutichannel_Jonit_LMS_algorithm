%% CLC 
clc      ;
clear    ;
close all;
%% Configuration 
fs = 10000   ;
T  = 0.2     ;
t  = 0:1/fs:T;
fre= 200     ;
Fw = 2*pi*fre;
Re = 1*sin(Fw*t);
%plot(Re(1:1024));
%grid on ;
%% Primary and Secondary path
P = zeros(125,1);
P(150)=1 ;
S = zeros(50,1);
S(25) = 1;
%% Filtering 
D   = filter(P,1,Re);% disturbance. 
F_x = filter(S,1,Re);% filtered signal.
%% FxLMS algorithm
L = 200;
muW = 0.0001;
noiseController = dsp.FilteredXLMSFilter('Length',L,'StepSize',muW, ...
    'SecondaryPathCoefficients',S);
[y,e] = noiseController(Re,D);
plot(e);
grid on ;
%% Tst Code
muW = 0.2;
obj = Fx_P1_LMS(L, S') ;
er = obj.controller(muW,Re,D);

figure ;
N= length(er);
plot(er)
grid on ;
figure ;
plot(1:N,e,1:N,er);
grid on ;

%% Tst Code
muW = 0.06;
obj = FxNLMS(L, S') ;
er = obj.controller(muW,Re,D);

figure ;
N= length(er);
plot(er)
grid on ;
figure ;
plot(1:N,e,1:N,er);
grid on ;
