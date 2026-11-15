function out = simulate_engagement(scenario, dt)
%SIMULATE_ENGAGEMENT  Closed-loop engagement, 0..10 s, fixed-step RK4 on the plant
%   with the controller output held (ZOH) over each step.
if nargin < 2, dt = 1e-3; end
P = guardian_params();
n = round(10/dt) + 1;
out.t = (0:n-1)'*dt;
f = {'x','y','los','theta','omega','i','V','miss','fire','valid'};
for j = 1:numel(f), out.(f{j}) = zeros(n,1); end
out.pulses = zeros(0,2);                      % [time, miss distance]
th = 0;  om = 0;  ic = 0;  prev_cmd = 0;  t_pulse = -Inf;
for k = 1:n
    t = out.t(k);
    [r_m, th_m, valid, tr] = target_motion_generator(t, scenario);
    [V, cmd] = turret_controller(t, r_m, th_m, valid, th, om);
    % miss distance = shortest distance from the target to the beam (a ray)
    along = tr.x*cos(th) + tr.y*sin(th);
    if along > 0, miss = abs(-tr.x*sin(th) + tr.y*cos(th)); else, miss = hypot(tr.x, tr.y); end
    % pulse logic: 0 -> 1 transition, honoured only after recharge
    if cmd == 1 && prev_cmd == 0 && (t - t_pulse) >= P.t_recharge
        t_pulse = t;  out.pulses(end+1,:) = [t, miss]; %#ok<AGROW>
        out.fire(k) = 1;
    end
    prev_cmd = cmd;
    out.x(k) = tr.x;  out.y(k) = tr.y;  out.los(k) = atan2(tr.y, tr.x);
    out.theta(k) = th;  out.omega(k) = om;  out.i(k) = ic;  out.V(k) = V;
    out.miss(k) = miss;  out.valid(k) = valid;
    % RK4 step of the plant
    s  = [th; om; ic];
    k1 = f_plant(t,        V, s);
    k2 = f_plant(t + dt/2, V, s + dt/2*k1);
    k3 = f_plant(t + dt/2, V, s + dt/2*k2);
    k4 = f_plant(t + dt,   V, s + dt*k3);
    s  = s + dt/6*(k1 + 2*k2 + 2*k3 + k4);
    th = s(1);  om = s(2);  ic = s(3);
    % mechanical elevation stops
    if th > P.th_max, th = P.th_max; om = min(om, 0); end
    if th < P.th_min, th = P.th_min; om = max(om, 0); end
end
% ---------------- performance measures ----------------
hit = out.pulses(:,2) <= P.R_eng;
out.n_pulses = size(out.pulses,1);
out.n_hits   = sum(hit);
if any(hit), out.t_intercept = out.pulses(find(hit,1),1); else, out.t_intercept = NaN; end
m = out.t > 3;                                 % after acquisition
out.rms_miss   = sqrt(mean(out.miss(m).^2));
out.max_miss   = max(out.miss(m));
out.avail      = mean(out.miss(m) <= P.R_eng); % fraction of time a pulse would hit
out.V_peak     = max(abs(out.V));
out.V_track    = max(abs(out.V(out.t > 2)));
out.i_peak     = max(abs(out.i));
end

function ds = f_plant(t, V, s)
[thd, thdd, id] = turret_plant(t, V, s(1), s(2), s(3));
ds = [thd; thdd; id];
end
