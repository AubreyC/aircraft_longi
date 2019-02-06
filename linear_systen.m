%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% *Aircraft Logitudinal Linear Analysis*
% 
% - Linearization around steady flight trim condition
% - SISO system for Elevator to Pitch  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;

%% Import Parameters
params;

%% Trim parameters
alpha_trim = 0.074030900438260;
v_a_trim = 70.000308594752980;
d_th_trim = 0.707469889617298;
d_elev_trim = -(P.Cm_0 + P.Cm_alpha*alpha_trim)/P.Cm_delta;

theta_trim = alpha_trim;
v_trim_bf = rot_Y(alpha_trim)*[v_a_trim;0;0];

%% Trim state
x_trim = [0;0; v_trim_bf(1);v_trim_bf(3);theta_trim;0];

u_trim = v_trim_bf(1);
w_trim = v_trim_bf(3);

q_trim = 0;

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

%% Elevator to Theta system
C = [0,0,0,1];
D = [0,0];
linear_sys = ss(A,-B,C,D);

%% Laplace transform: Input - Output form
tf_sys = tf(linear_sys);

%% PID controller for theta:
Kp_theta = 20;
Kd_theta = 8;
Ki_theta = 5;

tf_theta_controller = tf([Kd_theta Kp_theta Ki_theta],[1 0])

%% Feedback controller for theta
H = feedback(tf_theta_controller*tf_sys(1),1)

figure()
step(H)

figure()
margin(H)
