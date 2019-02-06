function [x_state_dot] = compute_state_deriv(x_state, f_bf, m_bf, P)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

p_n = x_state(1);
p_e = x_state(2);
p_d = x_state(3);

u = x_state(4);
v = x_state(5);
w = x_state(6);

phi = x_state(7);
theta = x_state(8);
psi = x_state(9);

p = x_state(10);
q = x_state(11);
r = x_state(12);


f_x = f_bf(1);
f_y = f_bf(2);
f_z = f_bf(3);

m_x = m_bf(1);
m_y = m_bf(2);
m_z = m_bf(3);

% Derivation only taking longitudinal dynamics:
p_n_dot = u*cos(theta) + w*sin(theta);
p_e_dot = 0;
p_d_dot = -u*sin(theta) + w*cos(theta);

u_dot = -q*w + (1/P.mass)*f_x;
v_dot = 0;
w_dot = q*u + (1/P.mass)*f_z;

p_dot = 0;
q_dot = (1/P.I_yy)*m_y;
r_dot = 0;

phi_dot = 0;
theta_dot = q;
psi_dot = 0;

x_state_dot = [p_n_dot; p_e_dot; p_d_dot; u_dot; v_dot; w_dot; phi_dot; theta_dot; psi_dot; p_dot; q_dot; r_dot];

end
