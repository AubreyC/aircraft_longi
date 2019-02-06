%% Parameters variables

%% Aero
P.g = 9.8;              % s^-2   
P.rho = 1.2682;         % kg/m^3;

%% Aircraft
P.thrust_max = 150000;  % N 
P.mass = 150*10^3;      % kg
P.Cl_0 = 0.9;           % no unit
P.Cl_alpha = 5.5;       % rad^-1
P.Cd_0 = 0.065;         % no unit
P.Cd_alpha = 0.4;       % rad^-1
P.Cm_0 = -0.3;          % no unit
P.Cm_alpha = -1.5;      % rad^-1
P.Cm_delta = -1.2;      % rad^-1
P.I_yy = 1.6*10^7;      % kg/m2
P.S = 360;              % m^2
P.c = 7.5;              % m

%% Controllers

% Pitch controller:
P.Kp_pitch = -15;
P.Kd_pitch = -8;
P.Ki_pitch = -1;

% Total Energy Control System (TECS):

% PI controller for Total Energy - Throttle control
P.Kp_energy_tot = 0.0000001;
P.Ki_energy_tot = 0.000000001;

% PID controller for Energy Balance - Theta control
P.Kp_energy_bal = 0.00000001;
P.Ki_energy_bal = 0.000000001;
P.Kd_energy_bal = 0.000000008;

P.energy_bal_coeff = 0.8;       % Controller the balance between altitude and speed control( 1=Priority on altitude / 0=Priority on speed)
P.alt_rate_limit = 5;           % Limit the change in altitude command - m/s
P.speed_rate_limit = 1;         % Limit the change in speed command - m/s^-2