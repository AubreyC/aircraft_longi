%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% *Aircraft Longitudinal Linear Analysis*
% 
% - Linearization around steady level flight trim condition
% - SISO analysis for Elevator to Pitch system
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;

%% Import Parameters
params;

%% Trim parameters

% v_a_trim = 70;
% [x_state_trim, v_a_trim, alpha_trim, d_th_trim, d_elev_trim] = find_trim_steady_level_flight(v_a_trim, P);

% Pre-computed value:
alpha_trim = 0.074030900438260;
v_a_trim = 70.000308594752980;
d_th_trim = 0.707469889617298;
d_elev_trim = -(P.Cm_0 + P.Cm_alpha*alpha_trim)/P.Cm_delta;

x_state_trim = define_state_trim(alpha_trim,d_th_trim, v_a_trim);

u_trim = x_state_trim(3);
w_trim = x_state_trim(4);
theta_trim = x_state_trim(5);
q_trim = x_state_trim(6);

%% Aerodynamic force coefficients
Cx_0 = -P.Cd_0;
Cx_alpha = (P.Cl_0 - P.Cd_alpha);

Cz_0 = -P.Cl_0;
Cz_alpha = -(P.Cd_0 + P.Cl_alpha);

%% Longitudinal linearization coefficients
% Ref: Small Unmanned Aircraft - Randal W. Beard & Tim McLain 

X_u = ((u_trim*P.rho*P.S)/P.mass)*(Cx_0 + Cx_alpha*alpha_trim) - (P.rho*P.S*w_trim*Cx_alpha)/(2*P.mass);
X_w = -q_trim + ((w_trim*P.rho*P.S)/P.mass)*(Cx_0 + Cx_alpha*alpha_trim) + (P.rho*P.S*u_trim*Cx_alpha)/(2*P.mass);
X_q = -w_trim;
X_de = 0;
X_dth = (1/P.mass)*P.thrust_max;

Z_u = q_trim + ((u_trim*P.rho*P.S)/P.mass)*(Cz_0 + Cz_alpha*alpha_trim) - (P.rho*P.S*w_trim*Cz_alpha)/(2*P.mass);
Z_w = ((w_trim*P.rho*P.S)/P.mass)*(Cz_0 + Cz_alpha*alpha_trim) +  (P.rho*P.S*u_trim*Cz_alpha)/(2*P.mass);
Z_q = u_trim;
Z_de = 0;

M_u = ((u_trim*P.rho*P.S*P.c)/P.I_yy)*(P.Cm_0 + P.Cm_alpha*alpha_trim + P.Cm_delta*d_elev_trim) - (P.rho*P.S*P.c*P.Cm_alpha*w_trim)/(2*P.I_yy);
M_w = ((w_trim*P.rho*P.S*P.c)/P.I_yy)*(P.Cm_0 + P.Cm_alpha*alpha_trim + P.Cm_delta*d_elev_trim) + (P.rho*P.S*P.c*P.Cm_alpha*u_trim)/(2*P.I_yy);
M_q = 0;
M_de = (P.rho*v_a_trim^2*P.S*P.c*P.Cm_delta)/(2*P.I_yy);


%% Linearized System around trim position
% x = [u; w; q; theta];
A = [X_u,                           X_w,                         X_q,                            -P.g*cos(theta_trim) ;...
     Z_u,                           Z_w,                         Z_q,                            -P.g*sin(theta_trim) ;...
     M_u,                           M_w,                         M_q,                             0                   ;...
       0,                             0,                           1,                             0                   ;];

B = [X_de, X_dth;...
     Z_de,     0;...
     M_de,     0;...
        0,     0];
 
disp('Eigen Values:');
eigen_values = eig(A)

if(real(eigen_values(4)))
    disp('/!\ Phugoid Mode is unstable /!\');
end

%% Elevator to Theta system
C = [0,0,0,1];
D = [0,0];
linear_sys = ss(A,B,C,D);

%% Laplace transform: Input - Output form for elevator to theta (pitch) system
tf_sys = tf(linear_sys);
theta_sys = tf_sys(1); 

%% PID Feedback controller for theta system
Kp_theta = -15;
Kd_theta = -8;
Ki_theta = -5;
tf_theta_controller = tf([Kd_theta Kp_theta Ki_theta],[1 0]);

H = feedback(tf_theta_controller*theta_sys,1)

figure()
step(H)
title('Pitch Controller Step response')

figure()
margin(H)
