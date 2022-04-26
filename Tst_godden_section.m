%% Clean memory and space 
close all ;
clc       ;
clear     ;

%% Testing function 

a   = 0 ;
b   = 3 ;
delta = 0.0238 ;


f =@fun ;
Lambda = Godden_section_search(f,a,b,delta);
disp("Lambda is : "+Lambda);
disp("Function value is :"+f(Lambda));
%% Function description 
function y=fun(x)
    y = 3*x^2 -4*x + 1;
end

