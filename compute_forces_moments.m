function [f_bf,m_bf] = compute_forces_moments(x_state, d_elev, d_th, P)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%% Parse state 
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

%% Compute angle of attack and airspeed
alpha = atan(w/u); % /!\ Should take the relative airspeed
V_a = norm([u;v;w]); % /!\ Should take the relative airspeed

%% Actuators limit:

d_th = max(min(d_th, 1),0); % Limit the thrust to [0:1]

% Limit the elevator delfection here:

%% Forces

% Lift and Drag
F_lift = 0.5*P.rho*V_a^2*P.S*(P.Cl_0 + P.Cl_alpha*alpha);
F_drag = 0.5*P.rho*V_a^2*P.S*(P.Cd_0 + P.Cd_alpha*alpha);
f_aero_bf = rot_Y(alpha)*[-F_drag;0; -F_lift];

% Gravity
f_grav_bf = rot_BF_NED(phi, theta, psi)*[0;0;P.mass*P.g];

% Thrust
f_trust_bf = [d_th*P.thrust_max; 0; 0]; % Thrust assumed to be in the X axis of the body frame

% Total forces in Body Frame
f_bf = f_aero_bf + f_grav_bf + f_trust_bf;

%% Moment
m_y = 0.5*P.rho*V_a^2*P.S*P.c*(P.Cm_0 + P.Cm_alpha*alpha + P.Cm_delta*d_elev);

m_bf = [0;m_y; 0];

end

