function [f_x_bf,f_z_bf, m_y_bf] = compute_forces_moments(x_state, d_elev, d_th, P, limit_actuator)
% Compute longitudinal Forces and Moments applied to the aircraft

%% Parse state 
p_n = x_state(1);   % North Position - Inertial NED frame
p_d = x_state(2);   % Down Position - Inertial NED frame
u = x_state(3);     % Velocity with respect to Inertial NED frame expressed in Body Frame along X
w = x_state(4);     % Velocity with respect to Inertial NED frame expressed in Body Frame along Z
theta = x_state(5); % Euler angle of the Body Frame With respect to the Inertial NED frame 
q = x_state(6);     % Angular rate of the body frame with respect to the inertial frame expressed in Body Frame


%% Compute angle of attack and airspeed: No wind
alpha = atan(w/u); % /!\ Should take the relative airspeed
V_a = norm([u;w]); % /!\ Should take the relative airspeed

%% Actuators limit:

if limit_actuator
    d_th = sat_value(d_th, 0,1); % Limit the thrust to [0:1]
%     d_elev = sat_value(d_elev, -1.0472, 1.0472);
end

% Limit the elevator delfection here:

%% Forces along X Z axis in Body Frame

% Lift and Drag
F_lift = 0.5*P.rho*V_a^2*P.S*(P.Cl_0 + P.Cl_alpha*alpha);
F_drag = 0.5*P.rho*V_a^2*P.S*(P.Cd_0 + P.Cd_alpha*alpha);
f_aero_bf = rot_Y(alpha)*[-F_drag;0; -F_lift];

% Gravity
f_grav_bf = rot_BF_NED(0, theta, 0)*[0;0;P.mass*P.g]; % Gravity is in NED frame

% Thrust
f_trust_bf = [d_th*P.thrust_max; 0; 0]; % Thrust assumed to be in the X axis of the body frame

% Total forces in Body Frame
f_bf = f_aero_bf + f_grav_bf + f_trust_bf;

f_x_bf = f_bf(1);
f_z_bf = f_bf(3);


%% Moment along Y axis in Body Frame

m_y_bf = 0.5*P.rho*V_a^2*P.S*P.c*(P.Cm_0 + P.Cm_alpha*alpha + P.Cm_delta*d_elev);

end

