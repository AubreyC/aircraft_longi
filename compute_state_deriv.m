function [x_state_dot] = compute_state_deriv(x_state, f_x_bf,f_z_bf, m_y_bf, P)
% Compute the longitudinal state derivatives according to the equation of
% motion


%% Parse state 
p_n = x_state(1);   % North Position - Inertial NED frame
p_d = x_state(2);   % Down Position - Inertial NED frame
u = x_state(3);     % Velocity with respect to Inertial NED frame expressed in Body Frame along X
w = x_state(4);     % Velocity with respect to Inertial NED frame expressed in Body Frame along Z
theta = x_state(5); % Euler angle of the Body Frame With respect to the Inertial NED frame 
q = x_state(6);     % Angular rate of the body frame with respect to the inertial frame expressed in Body Frame

%% Derivation only taking longitudinal dynamics
% Ref: Small Unnmanned Aircraft - Randal W. Beard

% Use rotation matrix to avoid error with sin/cos rotation:
rot_NED_BF = transpose(rot_BF_NED(0, theta, 0));
p_dot_NED = rot_NED_BF*[u;0;w];
p_n_dot = p_dot_NED(1);
p_d_dot = p_dot_NED(3);

u_dot = -q*w + (1/P.mass)*f_x_bf;
w_dot = q*u + (1/P.mass)*f_z_bf;

q_dot = (1/P.I_yy)*m_y_bf;

theta_dot = q;

%% State derivative
x_state_dot = [p_n_dot; p_d_dot; u_dot; w_dot; theta_dot; q_dot];

end
