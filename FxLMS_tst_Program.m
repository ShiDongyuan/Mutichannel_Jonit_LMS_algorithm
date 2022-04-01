%% CLC 
clc      ;
clear    ;
close all;
%% Configuration 
fs = 10000   ;
T  = 0.1     ;
t  = 0:1/fs:T;
fre= 200     ;
Fw = 2*pi*fre;
Re = sin(Fw*t);
%plot(Re(1:1024));
%grid on ;
%% Primary and Secondary path
P = zeros(125,1);
P(150)=1 ;
S = zeros(100,1);
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
FxLMS_B0(single(S),single(0),single(0),single(muW),int32(1));
N = length(t);
Yd = zeros(100,1);
er = zeros(N,1)  ;
%
e_in = 0 ;
for nn=1:N
    y = FxLMS_B0(single(S),single(Re(nn)),single(e_in),single(muW),int32(0));
    Yd=[y;Yd(1:end-1)];
    e_in   = D(nn)-Yd'*S;  
    er(nn) = e_in       ; 
end
figure ;
plot(er)
grid on ;
figure ;
plot(1:N,e,1:N,er);
legend({'FxLMS','Tst code'})
grid on ;
