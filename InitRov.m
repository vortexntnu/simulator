%% Global ROV parameters
m = 20;                                      % [kg] ROV mass
%I_g = ones(3,3);                             % [kg m^2] Inertia matrix about CG (symmetric!)
I_g = eye(3);
%% System matrices
M_RB_CG = [m*eye(3) zeros(3); zeros(3) I_g]; % [?] Rigid-body mass matrix in CG
r_bg = [0 0 0]';                             % [m] Translation vector from CG to CO (I think)
H = @(r) [eye(3) skew(r)'; zeros(3) eye(3)]; % [?] Transformation matrix from CG to CO
M_RB = H(r_bg)' * M_RB_CG * H(r_bg);         % [?] Rigid-body mass matrix in CO (should be transformed from M_RB_CG)

omega_b_bn = [0 0 0]';                                                   % [rad/s] Turn rate vector
C_RB_CG = [m*skew(omega_b_bn) zeros(3); zeros(3) -skew(I_g*omega_b_bn)]; % [?] Rigid-body Coriolis matrix in CG
C_RB = H(r_bg)' * C_RB_CG * H(r_bg);                                     % [?] Coriolis matrix in CO

% Find added mass matrix. Elements are calculated from tables. 
% Note! M_A > 0, so elements should have negative sign
omega_roll = -1;     % [rad/s] Natural frequency of roll
omega_pitch = -1;    % [rad/s] Natural frequency of pitch
Xu_0 = -1;           % [?] Added mass force coefficient in surge caused by acceleration in surge at 0 frequency
Yv_0 = -1;           % [?] Added mass force coefficient in sway  caused by acceleration in sway  at 0 frequency
Zw_0 = -1;           % [?] Added mass force coefficient in heave caused by acceleration in heave at 0 frequency
Kp_omega_roll = -1;  % [?] Added mass force coefficient in roll  caused by acceleration in roll  at natural frequency
Mq_omega_pitch = -1; % [?] Added mass force coefficient in pitch caused by acceleration in pitch at natural frequency
Nr_0 = -1;           % [?] Added mass force coefficient in yaw   caused by acceleration in yaw   at 0 frequency
M_A = -diag([Xu_0;
             Yv_0;
             Zw_0;
             Kp_omega_roll;
             Mq_omega_pitch;
             Nr_0           ]); % [?] Added mass matrix in CO (6x6)


random_input = rand(6,100);
time = 0:1/10:9.9;
nu_c    = timeseries(zeros(6,100), time);
tau     = timeseries(zeros(6,100), time);
tau_ext = timeseries(zeros(6,100), time);
%nu_0  = ones(6,1);
nu_0 = [5;1;0;1;0.5;0.5];
%eta_0 = zeros(6,1);
eta_0 = [0;0;0;0;0;0];
