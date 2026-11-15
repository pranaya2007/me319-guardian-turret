function [V_m, fire_cmd] = turret_controller(t, r_meas, theta_meas, meas_valid, theta_turret, theta_dot_turret) %#ok<INUSD>
%TURRET_CONTROLLER  Predictive-feedback fire control for the Guardian turret.
%
%   radar (r,theta) -> converted-measurement CA Kalman filter (x,vx,ax,y,vy,ay)
%                   -> prediction at horizon tp -> theta_ref = atan2(y_p, x_p)
%                   -> LTI compensator Gc(s) (delay-matched servo) -> V_m
%   fire_cmd is a one-call pulse issued when the CURRENT estimated miss distance
%   of the beam is inside the gate and the laser has recharged.
%
%   One fixed design for every scenario. All state is persistent and is reset
%   automatically when a new run starts (t = 0 or time jumps backwards).

persistent P S
if isempty(P), P = guardian_params(); end
if isempty(S) || t <= 0 || t < S.t_prev - 0.25
    S.t_prev  = t;
    S.init    = false;          % estimator initialised?
    S.x       = zeros(6,1);     % [x vx ax y vy ay]'
    S.Pm      = zeros(6);
    S.z_last  = [NaN NaN];      % last accepted radar sample (duplicate guard)
    S.xc      = zeros(2,1);     % compensator state
    S.e_prev  = 0;              % error held over the previous interval
    S.dt_c    = -1;  S.Ad = eye(2);  S.Bd = zeros(2,1);
    S.dt_last = 0;
    S.t_fire  = -Inf;
end
dt = max(t - S.t_prev, 0);
S.t_prev = max(t, S.t_prev);
if dt > 0, S.dt_last = dt; end

% ---------- 1. time update (runs every call, also through radar blackout) ----------
if S.init && dt > 0
    F1 = [1 dt dt^2/2; 0 1 dt; 0 0 1];
    Q1 = P.q*[dt^5/20 dt^4/8 dt^3/6; dt^4/8 dt^3/3 dt^2/2; dt^3/6 dt^2/2 dt];
    F  = blkdiag(F1, F1);
    S.x  = F*S.x;
    S.Pm = F*S.Pm*F' + blkdiag(Q1, Q1);
end

% ---------- 2. measurement update (only for a genuinely new radar sample) ----------
is_new = logical(meas_valid) && ~(r_meas == S.z_last(1) && theta_meas == S.z_last(2));
if is_new
    S.z_last = [r_meas theta_meas];
    c = cos(theta_meas);  s = sin(theta_meas);
    z  = [r_meas*c; r_meas*s];                      % converted measurement
    Jz = [c, -r_meas*s; s, r_meas*c];               % d(x,y)/d(r,theta)
    Rz = Jz*diag([P.sig_r^2, P.sig_th^2])*Jz';      % Cartesian noise covariance
    if ~S.init
        S.x  = [z(1); 0; 0; z(2); 0; 0];
        S.Pm = zeros(6);
        S.Pm([1 4],[1 4]) = Rz;
        S.Pm(2,2) = P.sig_v0^2;  S.Pm(5,5) = P.sig_v0^2;
        S.Pm(3,3) = P.sig_a0^2;  S.Pm(6,6) = P.sig_a0^2;
        S.init = true;
    else
        H  = [1 0 0 0 0 0; 0 0 0 1 0 0];
        Sk = H*S.Pm*H' + Rz;
        Kg = (S.Pm*H')/Sk;
        S.x  = S.x + Kg*(z - H*S.x);
        IKH  = eye(6) - Kg*H;
        S.Pm = IKH*S.Pm*IKH' + Kg*Rz*Kg';           % Joseph form
    end
    % physical-envelope projection: |v| <= v_max, |a| <= a_max
    vn = hypot(S.x(2), S.x(5));
    if vn > P.v_max, S.x([2 5]) = S.x([2 5])*(P.v_max/vn); end
    an = hypot(S.x(3), S.x(6));
    if an > P.a_max, S.x([3 6]) = S.x([3 6])*(P.a_max/an); end
end

if ~S.init                      % no radar data yet: stay put, hold fire
    V_m = 0;  fire_cmd = 0;  return
end

% ---------- 3. prediction and future elevation reference ----------
h  = P.tp + S.dt_last/2;        % tp ahead of the mid-point of the control hold
xp = S.x(1) + S.x(2)*h + S.x(3)*h^2/2;
yp = S.x(4) + S.x(5)*h + S.x(6)*h^2/2;
theta_ref = atan2(yp, xp);
theta_ref = min(max(theta_ref, P.th_min + P.ref_margin), P.th_max - P.ref_margin);

% ---------- 4. LTI compensator Gc(s), exact zero-order-hold discretisation ----------
if dt > 0
    if dt ~= S.dt_c
        E = expm([P.Ac P.Bc; zeros(1,3)]*dt);
        S.Ad = E(1:2,1:2);  S.Bd = E(1:2,3);  S.dt_c = dt;
    end
    S.xc = S.Ad*S.xc + S.Bd*S.e_prev;
end
e   = theta_ref - theta_turret;
V_m = P.Kc*(P.Cc*S.xc + e);
S.e_prev = e;

% ---------- 5. fire-control gate (uses the CURRENT estimate, not the prediction) ----------
n   = [-sin(theta_turret), cos(theta_turret)];      % unit normal to the beam
d   = n*[S.x(1); S.x(4)];                           % estimated signed miss distance
sd  = sqrt(max(n*S.Pm([1 4],[1 4])*n', 0));         % its 1-sigma uncertainty
fire_cmd = double(abs(d) <= P.d_gate && sd <= P.sig_gate && ...
                  (t - S.t_fire) >= P.t_recharge + 1e-6);
if fire_cmd, S.t_fire = t; end
end
