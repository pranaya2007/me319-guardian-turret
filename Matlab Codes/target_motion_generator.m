function [r_meas, theta_meas, meas_valid, truth] = target_motion_generator(t, scenario)
%TARGET_MOTION_GENERATOR  Radar measurements of the V-44 for a given scenario.
%   scenario: 'A' nominal | 'B' maneuver from t = 1 s | 'C' maneuver + radar loss
%             1.0-1.5 s | numeric id >= 100: seeded random admissible case.
%   Outputs follow the assignment interface. The optional 4th output 'truth'
%   (x,y,vx,vy,ax,ay) is for evaluation/plots only and is never given to the
%   controller. A new noisy sample is produced at 20 Hz; between samples the
%   last values are held and meas_valid = 0.
persistent P C
if isempty(P), P = guardian_params(); end
if isempty(C) || ~isequal(C.scenario, scenario) || t <= 0
    C = build_case(scenario, P);
    C.r = 0;  C.th = 0;
end
tr = truth_at(C, t);
k  = round(t/P.Ts_radar);
is_sample = abs(t - k*P.Ts_radar) < 1e-9;
meas_valid = 0;
if is_sample
    blackout = any(t >= C.bo(:,1) - 1e-9 & t < C.bo(:,2) - 1e-9);
    if ~blackout
        C.r  = hypot(tr.x, tr.y)  + P.sig_r *randn;
        C.th = atan2(tr.y, tr.x)  + P.sig_th*randn;
        meas_valid = 1;
    end
end
r_meas = C.r;  theta_meas = C.th;
if nargout > 3, truth = tr; end
end

% =====================================================================
function C = build_case(scenario, P)
C.scenario = scenario;
C.bo = zeros(0,2);
if ischar(scenario)
    C.kind = 1;                                   % analytic
    C.p0 = [1800; 600];  C.v0 = [-120; 0];
    switch upper(scenario)
        case 'A', C.A = [0; 0];     C.w = [1; 1];      seed = 11;
        case 'B', C.A = [-12; 14];  C.w = [1.5; 2.0];  seed = 12;
        case 'C', C.A = [-12; 14];  C.w = [1.5; 2.0];  seed = 13;  C.bo = [1.0 1.5];
        otherwise, error('unknown scenario');
    end
else
    C.kind = 2;                                   % tabulated random case
    seed = scenario;
    set_seed(seed);
    dt = 1e-3;  tt = 0:dt:10;  tau = max(tt - 1, 0);
    while true
        r0 = 1500 + 450*rand;  el = (8 + 42*rand)*pi/180;
        sp = 60 + 65*rand;     hd = el + pi + (rand - 0.5);
        a = zeros(2, numel(tt));
        for kk = 1:2
            for j = 1:3
                a(kk,:) = a(kk,:) + 7*rand*sin((0.5 + 3.5*rand)*tau);
            end
            t0 = 1 + 7*rand;
            a(kk,:) = a(kk,:) + (16*rand - 8)*tanh((tt - t0)/0.15).*(tt > 1).*(1 - exp(-tau/0.2));
        end
        if rand < 0.5                              % hard weave close to a_max
            w = 1.5 + 2.5*rand;  dirn = pi*rand;
            s = 19.5*tanh(3*sin(w*tau)).*(tt > 1).*(1 - exp(-tau/0.1));
            a = [cos(dirn)*s; sin(dirn)*s];
        end
        am = max(hypot(a(1,:), a(2,:)));
        if am > 19.9, a = a*(19.9/am)*(0.6 + 0.4*rand); end
        v = [sp*cos(hd) + cumsum(a(1,:))*dt; sp*sin(hd) + cumsum(a(2,:))*dt];
        p = [r0*cos(el) + cumsum(v(1,:))*dt; r0*sin(el) + cumsum(v(2,:))*dt];
        r = hypot(p(1,:), p(2,:));  elv = atan2(p(2,:), p(1,:));
        ok = min(r) >= P.r_min && max(r) <= P.r_max && ...
             max(hypot(v(1,:), v(2,:))) <= P.v_max && ...
             min(elv) >= -5*pi/180 && max(elv) <= 80*pi/180;
        if ok, break, end
    end
    C.tt = tt;  C.p = p;  C.v = v;  C.a = a;
    nb = floor(4*rand);                            % 0..3 radar-loss intervals
    for j = 1:nb
        s0 = 1 + 8*rand;
        C.bo(end+1,:) = [s0, s0 + 0.1 + 0.4*rand]; %#ok<AGROW>
    end
    seed = seed + 7;
end
set_seed(seed);                                    % reproducible radar noise
end

function tr = truth_at(C, t)
if C.kind == 1
    tau = max(t - 1, 0);  A = C.A;  w = C.w;
    acc = A.*sin(w*tau);
    vel = C.v0 + (A./w).*(1 - cos(w*tau));
    pos = C.p0 + C.v0*t + (A./w).*(tau - sin(w*tau)./w);
else
    k = min(max(round(t/1e-3) + 1, 1), numel(C.tt));
    pos = C.p(:,k);  vel = C.v(:,k);  acc = C.a(:,k);
end
tr.x = pos(1); tr.y = pos(2); tr.vx = vel(1); tr.vy = vel(2); tr.ax = acc(1); tr.ay = acc(2);
end

function set_seed(seed)
if exist('rng', 'file') || exist('rng', 'builtin')
    rng(seed, 'twister');
else                                               % older Octave
    rand('twister', seed);  randn('twister', seed); %#ok<RAND>
end
end
