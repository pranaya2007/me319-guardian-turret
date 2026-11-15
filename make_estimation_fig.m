function make_estimation_fig(scn)
% Estimation / prediction diagnostics (replica of the filter in turret_controller.m)
if nargin < 1, scn = 'C'; end
P  = guardian_params();
dt = 1e-3;  n = round(10/dt) + 1;  t = (0:n-1)'*dt;
o  = simulate_engagement(scn);                 % beam miss from the real closed loop
clear target_motion_generator                  % restart the same noise sequence

xt = zeros(n,1); yt = xt; thref = nan(n,1); sig = nan(n,1);
x = zeros(6,1); Pm = zeros(6); init = false;
H = [1 0 0 0 0 0; 0 0 0 1 0 0];
F1 = [1 dt dt^2/2; 0 1 dt; 0 0 1];
Q1 = P.q*[dt^5/20 dt^4/8 dt^3/6; dt^4/8 dt^3/3 dt^2/2; dt^3/6 dt^2/2 dt];
F  = blkdiag(F1, F1);  Q = blkdiag(Q1, Q1);

for k = 1:n
    [r, th, v, tr] = target_motion_generator(t(k), scn);
    xt(k) = tr.x;  yt(k) = tr.y;
    if init && k > 1
        x = F*x;  Pm = F*Pm*F' + Q;
    end
    if v
        c = cos(th);  s = sin(th);
        z  = [r*c; r*s];
        Jz = [c -r*s; s r*c];
        Rz = Jz*diag([P.sig_r^2 P.sig_th^2])*Jz';
        if ~init
            x  = [z(1); 0; 0; z(2); 0; 0];
            Pm = zeros(6);  Pm([1 4],[1 4]) = Rz;
            Pm(2,2) = P.sig_v0^2;  Pm(5,5) = P.sig_v0^2;
            Pm(3,3) = P.sig_a0^2;  Pm(6,6) = P.sig_a0^2;
            init = true;
        else
            Sk = H*Pm*H' + Rz;
            Kg = (Pm*H')/Sk;
            x  = x + Kg*(z - H*x);
            IKH = eye(6) - Kg*H;
            Pm = IKH*Pm*IKH' + Kg*Rz*Kg';
        end
        vn = hypot(x(2), x(5));
        if vn > P.v_max, x([2 5]) = x([2 5])*(P.v_max/vn); end
        an = hypot(x(3), x(6));
        if an > P.a_max, x([3 6]) = x([3 6])*(P.a_max/an); end
    end
    if init
        h  = P.tp + dt/2;
        xp = x(1) + x(2)*h + x(3)*h^2/2;
        yp = x(4) + x(5)*h + x(6)*h^2/2;
        thref(k) = atan2(yp, xp);
        thh = atan2(x(4), x(1));
        nv  = [-sin(thh), cos(thh)];
        sig(k) = sqrt(max(nv*Pm([1 4],[1 4])*nv', 0));
    end
end

kp = round(P.tp/dt);                           % samples in the prediction horizon
kk = (1:n-kp)';
thtrue = atan2(yt(kk+kp), xt(kk+kp));          % true LOS at t + tp
rr     = hypot(xt(kk+kp), yt(kk+kp));
perr   = rr.*(thref(kk) - thtrue);             % cross-LOS prediction error [m]

% ---------------- numbers for the paper ----------------
m3 = t(kk) > 3;
fprintf('prediction error, t>3 s: RMS %.2f m, max %.2f m\n', ...
        sqrt(mean(perr(m3).^2)), max(abs(perr(m3))));
fprintf('filter sigma_d, t>3 s: median %.2f m, max %.2f m\n', ...
        median(sig(t>3), 'omitnan'), max(sig(t>3), [], 'omitnan'));
if strcmpi(scn, 'C')
    bo = t >= 1 & t <= 2.5;
    fprintf('sigma_d in [1, 2.5] s: peak %.2f m, above the 0.6 m gate for %.2f s\n', ...
            max(sig(bo), [], 'omitnan'), sum(sig(bo) > P.sig_gate)*dt);
    kb = find(t > 1 & sig > P.sig_gate, 1);
    if isempty(kb)
        fprintf('sigma_d never exceeds the gate after t = 1 s\n');
    else
        fprintf('sigma_d first exceeds the gate at t = %.2f s\n', t(kb));
        kr = find(t > t(kb) & sig <= P.sig_gate, 1);
        if isempty(kr)
            fprintf('sigma_d does not return below the gate\n');
        else
            fprintf('sigma_d is back below the gate at t = %.2f s\n', t(kr));
        end
    end
    kk2 = (t(kk) >= 1 & t(kk) <= 2.5);
    fprintf('prediction error in [1, 2.5] s: max %.2f m\n', max(abs(perr(kk2))));
end

% ---------------- figure ----------------
figure('Color','w','Position',[50 50 700 650]);
subplot(3,1,1); plot(t(kk), perr, 'b'); hold on; grid on;
plot([0 10], P.R_eng*[1 1], 'k--', [0 10], -P.R_eng*[1 1], 'k--');
ylabel('prediction error [m]'); ylim([-10 10]); xlim([0 10]);
subplot(3,1,2); plot(t, sig, 'Color', [0 .5 0]); hold on; grid on;
plot([0 10], P.sig_gate*[1 1], 'k--');
ylabel('filter 1\sigma, cross-LOS [m]'); ylim([0 3]); xlim([0 10]);
subplot(3,1,3); semilogy(t, max(o.miss, 1e-3), 'r'); hold on; grid on;
plot([0 10], P.R_eng*[1 1], 'k--');
ylabel('beam miss [m]'); xlabel('t [s]'); ylim([1e-2 1e3]); xlim([0 10]);
if strcmpi(scn, 'C')
    for j = 1:3
        subplot(3,1,j); yl = ylim;
        patch([1 1.5 1.5 1], [yl(1) yl(1) yl(2) yl(2)], [.6 .6 .6], ...
              'FaceAlpha', .3, 'EdgeColor', 'none');
    end
end
save_fig('fig_estimation');
end