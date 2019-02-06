function [output] = func_trim(x, v_a_trim, P)
% Function to minimize in the optimization loop to find the trim values
% Compute difference between state derivative and state derivative desired
% for trim state

%% Parse input
alpha = x(1);
d_th = x(2);

%% Compute trim state derivative:

% Steady flight at level altitude (no climb)
x_trim_dot = [v_a_trim; 0; 0; 0; 0; 0];

% Steady flight at level altitude (no climb)
theta = alpha; 

% Compute velocity in Body frame
v_bf = rot_Y(alpha)*[v_a_trim;0;0];
u = v_bf(1);
w = v_bf(3);

% Define trim state - position are not important here 
x_state = [0;0;u;w;theta;0];

%% Compute Force and Moments

% Moment can be set to zero directly here
d_elev = -(P.Cm_0 + P.Cm_alpha*alpha)/P.Cm_delta;

% Compute Force and Moment
[f_x_bf,f_z_bf, m_y_bf] = compute_forces_moments(x_state, d_elev, d_th, P, false);

% Compute state derivative accordingly
x_state_dot_computed = compute_state_deriv(x_state, f_x_bf,f_z_bf, m_y_bf, P);

% Return difference which should be minimized
output = (x_state_dot_computed - x_trim_dot);

end

