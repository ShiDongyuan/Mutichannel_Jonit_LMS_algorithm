%% Drawing the root curve or the updating equation of the Joint LMS
H = tf([1],[1 -1 0 ],-1);
set(groot,'defaultAxesTickLabelInterpreter','latex')
rlocus(H)
zgrid
%%axis equal
xlabel('Real Axis','Interpreter','latex');
ylabel('Imaginary Axis','Interpreter','latex');
title('Root Locus','Interpreter','latex');
grid on ;
axis([-1 1 -1 1]);