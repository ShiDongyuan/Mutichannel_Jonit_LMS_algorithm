%% CLC 
clc      ;
clear    ;
close all;
%% Configuration 
fs = 10000   ;
T  = 3.8;
%%
% <http://www.mathworks.com MathWorks>    ;
t  = 0:1/fs:T;
fre= 111     ;
fre2 = 250 ;
Fw = 2*pi*fre;
Fw2 = 2*pi*fre2;
Re = awgn(4.2*sin(Fw*t) + 1.35*sin(Fw2*t),60);
%plot(Re(1:1024));
%grid on ;
%% Primary and Secondary path
P = zeros(125,1);
P(150)=1.3 ;
S = zeros(100,1);
S(25) = 0.8;
S(3)  = 2.2;
P = conv(P,S);
%% Filtering 
D   = filter(P,1,Re);% disturbance. 
F_x = filter(S,1,Re);% filtered signal.
%% FxLMS algorithm
L = 256;
muW = 0.000001;%0.0001
noiseController = dsp.FilteredXLMSFilter('Length',L,'StepSize',muW, ...
    'SecondaryPathCoefficients',S);
[y,e] = noiseController(Re,D);
plot(e);
grid on ;
%% Tst Code
muW = 0.01;
obj = Joint_LMS(L, S') ;
[er,obj] = obj.controller(muW,Re,D);

figure ;
N= length(er);
plot(er)
grid on ;
figure ;
plot(1:N,e,1:N,er);
grid on ;
wc_joint = obj.Wc;
%% Tst Code
muW = 0.01;
obj = FxNLMS(L, S') ;
[er,obj] = obj.controller(muW,Re,D);

wc_FxNLMS = obj.Wc;

figure ;
N= length(er);
plot(er)
grid on ;
figure ;
plot(1:N,e,1:N,er);
grid on ;

figure 
plot(1:length(wc_joint),wc_joint', 1:length(wc_FxNLMS),wc_FxNLMS');
grid on;