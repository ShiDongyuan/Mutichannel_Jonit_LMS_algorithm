%% CLC 
clc      ;
clear    ;
close all;
%% Configuration 
fs = 10000   ;
T  = 30.8;
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
Fc_low  = 800 ;
Fc_high = 2200 ;
Wc_low  = Fc_low*2/fs  ;
Wc_high = Fc_high*2/fs ;
FIR     = fir1(511,[Wc_low Wc_high]);
N = length(t);
Re = filter(FIR',1,randn(1,N));
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
muW = 0.001;%0.0001
noiseController = dsp.FilteredXLMSFilter('Length',L,'StepSize',muW, ...
    'SecondaryPathCoefficients',S);
[y,er] = noiseController(Re,D);
plot(er);
grid on ;

%% Tsting Multichannel Joint LMS 
A = FourChan_Joint_LMS_Element(L, length(S),S, S, S, S);

N  = length(Re);
Yd = zeros(length(S),1);
e  = 0         ;
Er = zeros(N,1);
for ii = 1:N
	[y1,y2,y3,y4,A] = A.controller(0.0001,0,0,0,Re(ii),0,0,0,e);
	Yd = [y4;Yd(1:end-1)];
	yt = S'*Yd           ;
	e  = D(ii)- yt       ; 
	Er(ii) = e           ;
end

figure 
plot(1:length(Er),Er,1:length(er),er);
grid on ;

figure 
plot(A.Wc_4)
grid on; 