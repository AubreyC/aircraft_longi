clear all;
% Import Parameters
params;

% 
% v_a_trim = 70; %m*s^-1
% alpha_trim = ((P.mass*P.g)/(0.5*P.rho*v_a_trim^2*P.S) - P.Cl_0)/P.Cl_alpha; 
% F_drag = 0.5*P.rho*v_a_trim^2*P.S*(P.Cd_0 + P.Cd_alpha*alpha_trim);
% d_th_trim = F_drag/P.thrust_max;
% d_elev_trim = -(P.Cm_0 + P.Cm_alpha*alpha_trim)/P.Cm_delta;
% 
% % Init state:
% theta_init = alpha_trim;
% 
% v_init_bf = rot_Y(alpha_trim)*[v_a_trim;0;0];
% x_init = [0;0;0; v_init_bf(1);v_init_bf(2);v_init_bf(3); 0;theta_init;0; 0;0;0];
% x_state = x_init;
% 
% wind = [0;0;0];
% [f_bf, m_bf] = compute_forces_moments(x_state, wind, d_elev_trim, d_th_trim, P)
% 
% trim_eq([alpha_trim, v_a_trim, d_th_trim],P)





f = @(x) trim_eq(x,P); % function of dummy variable y

alpha_init = 0.1;
V_a_init = 70;
d_th_init = 0.8;
x0 = [alpha_init, V_a_init, d_th_init];

options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','MaxFunctionEvaluations',1500)
[out,fval]=fsolve(f,x0, options)

