%DESIGN_ANALYSIS  Plant, compensator, stability margins and transient metrics.
%   Needs the Control System Toolbox (MATLAB) or 'pkg load control' (Octave).
if exist('OCTAVE_VERSION', 'builtin'), pkg load control; end
P = guardian_params();  s = tf('s');

Gp = P.Kp/(s*(s^2 + P.c1*s + P.c0));
Gc = P.Kc*(s^2 + P.c1*s + P.c0)/(s^2 + P.a*s + P.b_);
L  = minreal(Gc*Gp);   T = feedback(L, 1);   Sens = 1 - T;

fprintf('Gp(s) = %.4g / ( s (s^2 + %.4g s + %.5g) )\n', P.Kp, P.c1, P.c0);
wn = sqrt(P.c0);  fprintf('  electromechanical mode: wn = %.2f rad/s, zeta = %.4f\n', wn, P.c1/(2*wn));
fprintf('Gc(s) = %.4g (s^2 + %.4g s + %.5g) / (s^2 + %.4g s + %.4g),  Gc(0) = %.0f V/rad\n', ...
        P.Kc, P.c1, P.c0, P.a, P.b_, P.Kc*P.c0/P.b_);
fprintf('Routh:  a*b = %.0f  >  K = %.0f  (stable; K_min = 2/tp^3 = %.0f)\n', P.a*P.b_, P.K, 2/P.tp^3);
[Gm, Pm, Wcg, Wcp] = margin(L);
fprintf('GM = %.1f dB at %.2f rad/s,  PM = %.1f deg at %.2f rad/s,  delay margin = %.2f s\n', ...
        20*log10(Gm), Wcg, Pm, Wcp, Pm*pi/180/Wcp);
fprintf('Closed-loop poles: '); fprintf('%s  ', num2str(pole(T).', '%.2f')); fprintf('\n');
wv = logspace(-1, 2, 5000);  Tm = abs(squeeze(freqresp(T, wv)));
fprintf('Bandwidth = %.2f rad/s,  Kv = %.2f 1/s (= 1/tp),  Ms = %.2f\n', wv(find(Tm < 1/sqrt(2), 1)), P.K/P.b_, ...
        max(abs(squeeze(freqresp(Sens, wv)))));
[y, tt] = step(T, 0:1e-3:6);
fprintf('Step: overshoot = %.1f %%, t_100 = %.2f s, t_s(2%%) = %.2f s\n', ...
        100*(max(y) - 1), tt(find(y >= 1, 1)), tt(find(abs(y - 1) > 0.02, 1, 'last')));
fprintf('Residual delay-matching coefficient (tp^3/6 - 1/K) = %.4f s^3\n', P.tp^3/6 - 1/P.K);

figure(1); clf; margin(L); grid on;
save_fig('fig_bode');
figure(2); clf; rlocus(minreal(L/P.K)); title('Root locus of L(s)/K (design point K = 200)');
save_fig('fig_rlocus');
figure(3); clf;
w = logspace(-1, 2, 400);  Tw = squeeze(freqresp(T, w));
subplot(2,1,1); plot(tt, y, 'LineWidth', 1.2); hold on;
plot(tt, double(tt >= P.tp), 'k--'); grid on; xlabel('t [s]'); ylabel('step response');
legend('T(s)', 'ideal delay e^{-s t_p}', 'Location', 'southeast');
subplot(2,1,2); semilogx(w, -unwrap(angle(Tw))./w.', 'LineWidth', 1.2); hold on;
semilogx(w, P.tp*ones(size(w)), 'k--'); grid on; xlabel('\omega [rad/s]'); ylabel('phase delay [s]');
save_fig('fig_delay');
