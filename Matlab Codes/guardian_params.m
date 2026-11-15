function P = guardian_params()
%GUARDIAN_PARAMS  Single source of truth for every physical and design parameter.
%   Physical values are those prescribed in Section 2 of the assignment.
%   Design values are fixed once and used for ALL scenarios (no retuning).

% ---------------- physical parameters (prescribed) ----------------
P.Jt   = 1200;      % turret + cradle inertia about trunnion      [kg m^2]
P.Jm   = 0.20;      % motor rotor inertia                         [kg m^2]
P.N    = 100;       % gear ratio (motor speed / turret speed)     [-]
P.b    = 1500;      % turret-side viscous damping                 [N m s/rad]
P.Kt   = 10;        % motor torque constant                       [N m/A]
P.Ke   = 10;        % back-emf constant                           [V s/rad]
P.Rm   = 1.0;       % armature resistance                         [ohm]
P.Lm   = 0.10;      % armature inductance                         [H]
P.th_min = -10*pi/180;   % elevation limits                       [rad]
P.th_max =  85*pi/180;
P.Ts_radar = 0.05;  % radar interval (20 Hz)                      [s]
P.sig_r  = 1.0;             % range noise std                     [m]
P.sig_th = 0.02*pi/180;     % elevation noise std                 [rad]
P.tp     = 0.50;    % prescribed prediction horizon               [s]
P.t_recharge = 1.0; % minimum inter-pulse interval                [s]
P.R_eng  = 2.0;     % engagement radius                           [m]
P.v_max  = 150;  P.a_max = 20;  P.r_min = 800;  P.r_max = 2000;

% ---------------- design parameters (fixed once, never retuned) ----------------
P.K        = 200;   % loop gain of L(s) = K/(s(s^2+a s+b)); a, b follow from tp
P.q        = 1000;  % white-noise-jerk PSD of the CA Kalman filter [m^2/s^5]
P.sig_v0   = 100;   % initial velocity uncertainty (< v_max)       [m/s]
P.sig_a0   = 12;    % initial acceleration uncertainty (< a_max)   [m/s^2]
P.d_gate   = 0.8;   % fire gate: |estimated miss distance|         [m]
P.sig_gate = 0.6;   % fire gate: max 1-sigma cross-range error     [m]
P.ref_margin = 0.5*pi/180;   % reference kept this far inside the hard stops

% Optional override, used ONLY by tune_study.m for the sensitivity study.
global GUARDIAN_OVERRIDE %#ok<GVMIS>
if isstruct(GUARDIAN_OVERRIDE)
    fn = fieldnames(GUARDIAN_OVERRIDE);
    for j = 1:numel(fn), P.(fn{j}) = GUARDIAN_OVERRIDE.(fn{j}); end
end

% ---------------- plant  Gp(s) = Kp / ( s (s^2 + c1 s + c0) ) ----------------
P.J  = P.Jt + P.N^2*P.Jm;                               % reflected inertia = 3200
a2   = P.Lm*P.J;
a1   = P.Lm*P.b + P.Rm*P.J;
a0   = P.Rm*P.b + P.N^2*P.Kt*P.Ke;
P.Kp = P.N*P.Kt/a2;                                     % 3.125
P.c1 = a1/a2;                                           % 10.469
P.c0 = a0/a2;                                           % 3129.7

% ---------------- controller (delay-matched servo) ----------------
% Target loop  L(s) = K / ( s (s^2 + a s + b) ),  T(s) = K/(s^3 + a s^2 + b s + K)
% Matching T(s) to exp(-s tp) through O(s^2):  b = K tp,  a = K tp^2/2.
% Routh:  a b > K  <=>  K > 2/tp^3 = 16.   K = 200 -> L = 200/(s (s+5)(s+20)).
P.a  = P.K*P.tp^2/2;                                    % 25
P.b_ = P.K*P.tp;                                        % 100
P.Kc = P.K/P.Kp;                                        % 64 V/rad (HF gain)
% Gc(s) = Kc (s^2 + c1 s + c0)/(s^2 + a s + b) = Kc [ 1 + Cc (sI - Ac)^-1 Bc ]
P.Ac = [0 1; -P.b_ -P.a];
P.Bc = [0; 1];
P.Cc = [P.c0 - P.b_,  P.c1 - P.a];
end
