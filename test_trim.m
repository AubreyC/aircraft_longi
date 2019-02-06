clear all;
% Import Parameters
params;


f = @(x) trim_eq(x,P); % function of dummy variable y

alpha_init = 0.1;
V_a_init = 70;
d_th_init = 0.8;
x0 = [alpha_init, V_a_init, d_th_init];

options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','MaxFunctionEvaluations',1500)
[out,fval]=fsolve(f,x0, options)

