function [x_state_trim, v_a_trim, alpha_trim, d_th_trim, d_elev_trim] = find_trim_steady_level_flight(v_a_trim, P)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% *Aircraft Logitudinal Trim*
% 
% - Steady flight at level altitude (no climb)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Optimization routine

% Define function to minimize
% x = [alpha, throttle]
f = @(x) func_trim(x,v_a_trim, P); % function of dummy variable y

% Define init input
alpha_init = 0.1;
d_th_init = 0.8;
x0 = [alpha_init, d_th_init];

% Run the optimization
options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','MaxFunctionEvaluations',1500);
[out,fval]=fsolve(f,x0, options);

if fval > 10^-5
    disp('Warning trim values might erroneous')
end

%% Results
disp('Trim Result for steady flught at level altitude:')
alpha_trim = out(1)
d_th_trim = out(2)
d_elev_trim = -(P.Cm_0 + P.Cm_alpha*alpha_trim)/P.Cm_delta

x_state_trim = define_state_trim(alpha_trim,d_th_trim, v_a_trim)


end
