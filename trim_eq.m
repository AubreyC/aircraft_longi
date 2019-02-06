function [f_bf] = trim_eq(x, P)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here

alpha = x(1);
V_a = x(2);
d_th = x(3);

phi = 0;
theta = alpha;
psi = 0;

% Lift and Drag
F_lift = 0.5*P.rho*V_a^2*P.S*(P.Cl_0 + P.Cl_alpha*alpha);
F_drag = 0.5*P.rho*V_a^2*P.S*(P.Cd_0 + P.Cd_alpha*alpha);
f_aero_xz = [cos(alpha) -sin(alpha); sin(alpha) cos(alpha)]*[-F_drag; -F_lift];
f_aero_bf = [f_aero_xz(1);0; f_aero_xz(2)];

% f_aero_bf = rot_BF_NED(phi, theta, psi)*[-F_drag;0;-F_lift];

% Gravity
f_grav_bf = rot_BF_NED(phi, theta, psi)*[0;0;P.mass*P.g];

% Thrust
f_trust_bf = [d_th*P.thrust_max; 0; 0];

f_bf = f_aero_bf + f_grav_bf + f_trust_bf;

end

