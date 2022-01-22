%% Comparison between the multichannel Join LMS and McFxLMS algorithms

%% Introduction
% We simulated a fully-connected four-channel MCANC system using the Multichannel joint LMS and McFxLMS algorithms in this simulation. The first figure depicts the frequency responses of sixteen control filters used in these two algorithms, while the second figure depicts four error signals.
% Notably, both algorithms use the same step size of 0.000013. As demonstrated by the results, the McFxLMS algorithm converges faster than the multichannel Joint LMS algorithm. However, the Joint LMS algorithm achieves nearly identical control filters to the McFxLMS algorithm and would significantly reduces computations.
%
% <<1-s2.0-S0165168408003770-gr1.jpg>>
%
% The algorithms' source code is available on Github:
% <https://github.com/ShiDongyuan/Multichannel_FxLMS_python_code.git McFxLMS>
%
%% Clean memory and space
clc       ;
close all ;
clear     ;


% Loading the control filter from space
a = load('Four_channel_ANC_tst_Program.mat') ;
b = load('Tst_4channel_program_McFxLMS.mat');
fs = 16000;

set(groot,'defaultAxesTickLabelInterpreter','latex')
b_1 = permute(b.Wc_matrix,[3 1 2]);

%% Drawing the control filters of different algorithms
L = 512 ;
for jj = 1:4
    figure
    for ii = 1:4
        Wf11  = fft(a.Wc_matrix(:,jj,ii));
        Wnf11 = fft(b_1(:,1,ii));
        f     = fs*(0:(L/2))/L;
        Pf11  = abs(Wf11).^2;
        Pnf11 = abs(Wnf11).^2;
        subplot(2,2,ii)
        plot(f,Pf11(1:L/2+1),f,Pnf11(1:L/2+1));
        axis([-inf 6000, -inf inf]);
        titxt = sprintf('$\\|W_{%d%d}(f)\\|^2$',ii,jj);
        title(titxt,'Interpreter','latex');
        xlabel('Frequency (Hz)','Interpreter','latex');
        ylabel('Magnitude','Interpreter','latex');
        grid on
        if ii==1
            legend({'Multichannel joint LMS','McFxLMS'},'Interpreter','latex');
        end
    end
end

%plot(1:512, a.Wc_matrix(:,2,3), 1:512, b_1(:,2,3));
grid on ;

%% Drawing the four error signals
figure
for ii = 1:4
    subplot(2,2,ii);
    index = 1:length(a.Err_v(:,ii));
    plot(index/fs, a.Err_v(:,ii),index/fs, b.Err_array(:,ii));
    grid on ;
    axis([-inf inf -inf inf]);
    title("The " + num2str(ii)+"th error sginal",'Interpreter','latex');
    if ii==1
        legend({'Multichannel joint LMS','McFxLMS'},'Interpreter','latex');
    end
    xlabel('Time (second)','Interpreter','latex');
end