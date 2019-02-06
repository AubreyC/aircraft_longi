%% Linear State Space system analysis

clear all;
params;


v_a_trim = 80; %m*s^-1
alpha_trim = ((P.mass*P.g)/(0.5*P.rho*v_a_trim^2*P.S) - P.Cl_0)/P.Cl_alpha; 


F_drag = 0.5*P.rho*v_a_trim^2*P.S*(P.Cd_0 + P.Cd_alpha*alpha_trim);
d_th_trim = F_drag/P.thrust_max;

d_elev_trim = -(P.Cm_0 + P.Cm_alpha*alpha_trim)/P.Cm_delta;

% Init state:
theta_trim = alpha_trim;

v_trim_bf = rot_Y(alpha_trim)*[v_a_trim;0;0];
Va_trim = norm(v_trim_bf);

u_trim = v_trim_bf(1);
w_trim = v_trim_bf(3);

q_trim = 0;

Cx_0 = -P.Cd_0;
Cx_alpha = (P.Cl_0 - P.Cd_alpha);

Cz_0 = -P.Cl_0;
Cz_alpha = -(P.Cd_0 + P.Cl_alpha);


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
M_de = (P.rho*Va_trim^2*P.S*P.c*P.Cm_delta)/(2*P.I_yy);


% A = [X_u,                           X_w*Va_trim*cos(alpha_trim), X_q,                            -P.g*cos(theta_trim)                           ;...
%      Z_u/(Va_trim*cos(alpha_trim)), Z_w,                         Z_q/(Va_trim*cos(alpha_trim)), (-P.g*sin(theta_trim))/(Va_trim*cos(alpha_trim));...
%      M_u,                           M_w*Va_trim*cos(alpha_trim), M_q,                             0                                             ;...
%      0,                             0,                           1,                               0                                             ;];


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
 
C = [0,0,0,1];
D = [0,0];
 
linear_sys = ss(A,-B,C,D);

%% Laplace transform: Input - Output form
tf_sys = tf(linear_sys);
figure()
rlocus(tf_sys(1))

Kp_theta = 15;
Kd_theta = 8;
Ki_theta = 1;

tf_theta_controller = tf([Kd_theta Kp_theta Ki_theta],[1 0])

H = feedback(tf_theta_controller*tf_sys(1),1)

figure()
step(H)

figure()
[Gm,Pm] = margin(H)

figure()
rlocus(H)
