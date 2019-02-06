clear all;

%% Import Parameters
params;

%% Similation parameters
delta_s = 0.01; % Simulation time step - seconds
time_f = 1000; % Final time  - seconds
time_steps = 0:delta_s:time_f;
nb_steps = time_f/delta_s  + 1;

%% Compute trim state:

alpha_trim = 0.074030900438260;
v_a_trim = 70.000308594752980;
d_th_trim = 0.707469889617298;
d_elev_trim = -(P.Cm_0 + P.Cm_alpha*alpha_trim)/P.Cm_delta;

% Init state at trim:
theta_init = alpha_trim;
theta_trim = theta_init;

v_init_bf = rot_Y(alpha_trim)*[v_a_trim;0;0];
x_init = [0;0;0; v_init_bf(1);v_init_bf(2);v_init_bf(3); 0;theta_init;0; 0;0;0];
x_state = x_init;

%% Logging varialbles:
x_state_log = zeros(12,nb_steps);
theta_ref_log = zeros(1,nb_steps);
alt_ref_log = zeros(1,nb_steps);
v_ref_log = zeros(1,nb_steps);

E_bal_e_log = zeros(1,nb_steps);
E_tot_e_log = zeros(1,nb_steps);

E_pot_log = zeros(1,nb_steps);
E_kin_log = zeros(1,nb_steps);

E_pot_c_log = zeros(1,nb_steps);
E_kin_c_log = zeros(1,nb_steps);

theta_c_Kp_log = zeros(1,nb_steps);
theta_c_Ki_log = zeros(1,nb_steps);
theta_c_Kd_log = zeros(1,nb_steps);

d_th_Ki_log = zeros(1,nb_steps);
d_th_Kp_log = zeros(1,nb_steps);

d_elev_total_log = zeros(1,nb_steps);
d_th_total_log = zeros(1,nb_steps);

%% Define reference trajectory here:
% - Reference Altitude
% - Reference Speed

alt_ref = zeros(1,nb_steps);
alt_ref(:,1:floor(nb_steps/2)) = 0;
alt_ref(:,floor(nb_steps/2):end) = 50; 

v_ref = zeros(1,nb_steps);
v_ref(:,1:floor(nb_steps/2)) = 70;
v_ref(:,floor(nb_steps/2):end) = 75;

%% Controllers variables initialization
integral_theta_error = 0;
integral_energy_tot_error = 0;
integral_energy_bal_error = 0;

alt_c = alt_ref(1,1);
V_c = v_ref(1,1);
theta_c = theta_trim;

E_bal_e_last = nan;

%% Simulation loop
index = 1;
for time = time_steps
    
    % Logging states
    x_state_log(:,index) = x_state;
    index = index + 1;
    
    %% TECS controller
    % Ref: Total Energy Control for LongitudinalAutopilot - Randal W. Beard
    % http://uavbook.byu.edu/lib/exe/fetch.php?media=shared:tecs_autopilot.pdf
    
    % Current potential and kinematics energies:
    alt = -x_state(3); 
    V = norm(x_state(4:6));

    E_pot = P.mass*P.g*alt;
    E_kin = 0.5*P.mass*V^2;
    
    E_pot_log(1, index-1) = E_pot;
    E_kin_log(1, index-1) = E_kin;

    E_tot = E_pot + E_kin;

    % Target potential and kinematics energy
    alt_c = compute_alt_command(alt_c, alt_ref(1,index-1), 5, delta_s);
    alt_ref_log(1,index-1) = alt_c;
    
    V_c = compute_alt_command(V_c, v_ref(1,index-1), 7, delta_s);
    v_ref_log(1,index-1) = V_c;
    
    E_pot_c = P.mass*P.g*alt_c;
    E_kin_c = 0.5*P.mass*V_c^2;
    
    E_pot_c_log(1, index-1) = E_pot_c;
    E_kin_c_log(1, index-1) = E_kin_c;
    
    % Error on potential and kinemtic energy
    E_pot_e = E_pot_c - E_pot;
    E_pot_e = sat_value(E_pot_e, -P.mass*P.g*50, P.mass*P.g*50);

    E_kin_e = E_kin_c - E_kin;
    E_kin_e = sat_value(E_kin_e, -0.5*P.mass*50, +0.5*P.mass*50);
    
    E_tot_e = E_pot_e + E_kin_e;
    E_bal_e =  2*(P.energy_bal_coeff*E_pot_e - (1-P.energy_bal_coeff)*E_kin_e);
    
    E_bal_e =  sat_value(E_bal_e, -1/P.Kp_energy_bal, 1/P.Kp_energy_bal);
    E_tot_e =  sat_value(E_tot_e, -1/P.Kp_energy_tot, 1/P.Kp_energy_tot);

    E_bal_e_log(1,index-1) = E_bal_e;
    E_tot_e_log(1,index-1) = E_tot_e;
    
    if isnan(E_bal_e_last)
        E_bal_e_last = E_bal_e;
    end;
    
    E_bal_e_deriv = (E_bal_e - E_bal_e_last)/delta_s;
    E_bal_e_last = E_bal_e;

    % Error integrals
    integral_energy_bal_error = integral_energy_bal_error + E_bal_e*delta_s;
    integral_energy_bal_error = min((1/P.Ki_energy_bal), max(integral_energy_bal_error, (-1/P.Ki_energy_bal)));

    integral_energy_tot_error = integral_energy_tot_error + E_tot_e*delta_s;
    integral_energy_tot_error = min((1/P.Ki_energy_tot), max(integral_energy_tot_error, (-1/P.Ki_energy_tot)));

    % PI controller on total energy and energy balance
    theta_c = P.Kp_energy_bal*E_bal_e + P.Kd_energy_bal*E_bal_e_deriv + P.Ki_energy_bal*integral_energy_bal_error + theta_trim;
        
    theta_c_Kp_log(1, index-1) = P.Kp_energy_bal*E_bal_e;
    theta_c_Ki_log(1, index-1) = P.Ki_energy_bal*integral_energy_bal_error;
    theta_c_Kd_log(1, index-1) = P.Kd_energy_bal*E_bal_e_deriv;

    d_th = P.Kp_energy_tot*E_tot_e + P.Ki_energy_tot*integral_energy_tot_error;
    d_th_Kp_log(1, index-1) = P.Kp_energy_tot*E_tot_e;
    d_th_Ki_log(1, index-1) = P.Ki_energy_tot*integral_energy_tot_error;

    %% Pitch Controller
    theta_ref_log(1,index-1) = theta_c;
    
    % Pitch error
    theta_error = theta_c - x_state(8);
    integral_theta_error = integral_theta_error + theta_error*delta_s;
    theta_dot_error = - x_state(11);
    
    % PID controller on theta
    d_elev = P.Kp_pitch*theta_error + P.Kd_pitch*theta_dot_error + P.Ki_pitch*integral_theta_error;
    
    %% Compute dynamics
    
    % Input deflection from trim state:
    d_elev_total = d_elev_trim + d_elev;
    d_th_total = d_th_trim + d_th;
    d_th_total = max(min(d_th_total, 1),0);

    d_elev_total_log(1, index-1) = d_elev_total;
    d_th_total_log(1, index-1) = d_th_total;

    % Compute Forces and Moments in Body Frame
    [f_bf, m_bf] = compute_forces_moments(x_state, d_elev_total, d_th_total, P);

    % Compute dynamics according to the equations of motion
    x_state_dot = compute_state_deriv(x_state, f_bf, m_bf, P);
    x_state = x_state + x_state_dot*delta_s; % Raw integration

end

%% Plotting

figure();
subplot(3,1,1);
plot(time_steps,x_state_log(1,:));
xlabel('Time (s)');
ylabel('pn (m)');
title('Position North')

subplot(3,1,2);
plot(time_steps,-x_state_log(3,:));
xlabel('Time (s)');
ylabel('Alt (m)');
title('Altitude')

subplot(3,1,3);
plot(time_steps,sqrt(x_state_log(4,:).*x_state_log(4,:) + x_state_log(5,:).*x_state_log(5,:) + x_state_log(6,:).*x_state_log(6,:)));
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity')


figure();
subplot(3,1,1);
plot(time_steps,x_state_log(4,:));
xlabel('Time (s)');
ylabel('u (m/s)');
title('Velocity x bf')

subplot(3,1,2);
plot(time_steps,x_state_log(6,:));
xlabel('Time (s)');
ylabel('w (m/s)');
title('Velocity w bf')

subplot(3,1,3);
plot(time_steps,(x_state_log(8,:)));
xlabel('Time (s)');
ylabel('Theta (rad)');
title('Theta (rad)')

figure();
subplot(3,1,1);
plot(time_steps,x_state_log(11,:));
xlabel('Time (s)');
ylabel('q (rad/s)');
title('Angle rate y bf')

subplot(3,1,2);
plot(time_steps,atan2(x_state_log(6,:),x_state_log(4,:)));
xlabel('Time (s)');
ylabel('Alpha (rad)');
title('AoA (rad)')

subplot(3,1,3);
plot(time_steps,x_state_log(8,:)-atan2(x_state_log(6,:),x_state_log(4,:)));
xlabel('Time (s)');
ylabel('Gamma (rad)');
title('Gamma (rad)')


figure();
subplot(3,1,1);
plot(time_steps,-x_state_log(3,:),'b');
hold on
plot(time_steps,alt_ref_log(1,:),'r');
hold on
plot(time_steps,alt_ref,'g');

xlabel('Time (s)');
ylabel('Altitude (m)');
legend('current', 'target','ref')
title('Altitude')


subplot(3,1,2);
plot(time_steps,sqrt(x_state_log(4,:).*x_state_log(4,:) + x_state_log(5,:).*x_state_log(5,:) + x_state_log(6,:).*x_state_log(6,:)),'b');
hold on
plot(time_steps,v_ref_log,'r');
hold on
plot(time_steps,v_ref,'g');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('current', 'target','ref')
title('Velocity')

subplot(3,1,3);
plot(time_steps,(x_state_log(8,:)),'b');
hold on
plot(time_steps,theta_ref_log(1,:),'r');
xlabel('Time (s)');
ylabel('Theta (rad)');
legend('current', 'target')
title('Theta')


figure()
subplot(4,1,1);
plot(time_steps,E_tot_e_log(1,:),'b');
xlabel('Time (s)');
ylabel('Energy total error');
title('Energy total error')

subplot(4,1,2);
plot(time_steps,E_bal_e_log(1,:),'b');
xlabel('Time (s)');
ylabel('Energy balance error');
title('Energy balance error')

subplot(4,1,3);
plot(time_steps,E_pot_log(1,:),'b');
hold on
plot(time_steps,E_pot_c_log(1,:),'r');
xlabel('Time (s)');
ylabel('Energy potential');
legend('current', 'target')
title('Energy potential')

subplot(4,1,4);
plot(time_steps,E_kin_log(1,:),'b');
hold on
plot(time_steps,E_kin_c_log(1,:),'r');
xlabel('Time (s)');
ylabel('Energy kinetic');
legend('current', 'target')
title('Energy kinetic')

figure()
subplot(3,1,1);
plot(time_steps,d_th_total_log(1,:),'b');
xlabel('Time (s)');
ylabel('Throttle command');
title('Throttle command')

subplot(3,1,2);
plot(time_steps,d_elev_total_log(1,:),'b');
xlabel('Time (s)');
ylabel('Elevator command');
title('Elevator command')

subplot(3,1,3);
plot(time_steps,(x_state_log(8,:)),'b');
hold on
plot(time_steps,theta_ref_log(1,:),'r');
xlabel('Time (s)');
ylabel('Theta (rad)');
legend('current', 'target')
title('Theta')


figure()
subplot(2,1,1);
plot(time_steps,d_th_Kp_log(1,:),'b');
hold on
plot(time_steps,d_th_Ki_log(1,:),'r');
xlabel('Time (s)')
ylabel('TECS Th control');
legend('Kp', 'Ki')
title('TECS: Throttle controller')

subplot(2,1,2);
plot(time_steps,theta_c_Kp_log(1,:),'b');
hold on
plot(time_steps,theta_c_Ki_log(1,:),'r');
hold on 
plot(time_steps,theta_c_Kd_log(1,:),'g');
xlabel('Time (s)')
ylabel('TECS theta control');
legend('Kp', 'Ki', 'Kd')
title('TECS: Theta controller')
