function [x_state_trim] = define_state_trim(alpha_trim,d_th_trim, v_a_trim)
% Define the trim state for steady flight at level altitude from the computed
% - alpha_trim
% - d_th_trim
% - v_a_trim

% Steady flught at level altitude: 
theta_trim = alpha_trim;

v_trim_bf = rot_Y(alpha_trim)*[v_a_trim;0;0];
u_trim = v_trim_bf(1);
w_trim = v_trim_bf(3);

x_state_trim = [0;0;u_trim;w_trim;theta_trim;0];

end

