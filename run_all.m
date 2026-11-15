%RUN_ALL  Reproduces every number and figure in the paper.
%   1) standardized scenarios A, B, C     2) Monte-Carlo generalisation study
clear functions; close all; clc;
P = guardian_params();
if exist('results.txt', 'file'), delete('results.txt'); end
diary('results.txt');

% ---------------- 1. standardized scenarios ----------------
sc = {'A', 'B', 'C'};
fprintf('Scen  t_int[s]  pulses  hits  rms[m]  max[m]  avail  Vpk[V]  Vtrk[V]  ipk[A]\n');
figure(1); clf; set(gcf, 'Position', [50 50 1100 650]);
for j = 1:3
    o = simulate_engagement(sc{j});  R{j} = o; %#ok<SAGROW>
    fprintf('%s     %6.3f   %4d   %4d   %5.2f   %5.2f   %5.3f  %6.0f  %6.0f  %6.1f\n', sc{j}, ...
       o.t_intercept, o.n_pulses, o.n_hits, o.rms_miss, o.max_miss, o.avail, o.V_peak, o.V_track, o.i_peak);
    subplot(3,3,j);
    plot(o.t, o.los*180/pi, 'k', o.t, o.theta*180/pi, 'r--', 'LineWidth', 1.1); grid on;
    title(['Scenario ' sc{j}]); ylabel('elevation [deg]');
    if j == 1, legend('true LOS', 'turret', 'Location', 'northwest'); end
    subplot(3,3,3+j);
    semilogy(o.t, max(o.miss, 1e-3), 'b'); hold on; grid on;
    plot([0 10], P.R_eng*[1 1], 'k--');
    plot(o.pulses(:,1), max(o.pulses(:,2), 1e-3), 'rv', 'MarkerFaceColor', 'r');
    ylabel('miss distance [m]'); ylim([1e-2 1e3]);
    if j == 3, patch([1 1.5 1.5 1], [1e-2 1e-2 1e3 1e3], [.6 .6 .6], 'FaceAlpha', .3, 'EdgeColor', 'none'); end
    subplot(3,3,6+j);
    plot(o.t, o.V, 'Color', [0 .5 0]); grid on; xlabel('t [s]'); ylabel('V_m [V]');
end
save_fig('fig_scenarios');

figure(2); clf;
plot(R{1}.x, R{1}.y, 'k', R{2}.x, R{2}.y, 'b', 'LineWidth', 1.2); hold on; grid on; axis equal;
plot(0, 0, 'rs', 'MarkerFaceColor', 'r');
th = linspace(0, pi/2, 100); plot(800*cos(th), 800*sin(th), 'k:', 2000*cos(th), 2000*sin(th), 'k:');
xlabel('x [m]'); ylabel('y [m]'); legend('A (nominal)', 'B/C (maneuver)', 'turret');
save_fig('fig_traj');

% ---------------- 2. Monte-Carlo: random admissible maneuvers + radar losses ----------------
Nmc = 200;  tint = nan(Nmc,1);  np_ = zeros(Nmc,1);  nh = zeros(Nmc,1);  allmiss = [];
for m = 1:Nmc
    o = simulate_engagement(1000 + m);
    tint(m) = o.t_intercept;  np_(m) = o.n_pulses;  nh(m) = o.n_hits;
    allmiss = [allmiss; o.pulses(:,2)]; %#ok<AGROW>
end
fprintf('\nMonte-Carlo (%d random cases)\n', Nmc);
fprintf('  interception success    : %.1f %%\n', 100*mean(~isnan(tint)));
fprintf('  interception time       : mean %.3f s, max %.3f s\n', mean(tint(~isnan(tint))), max(tint));
fprintf('  pulses per engagement   : %.2f,  pulse hit ratio %.1f %%\n', mean(np_), 100*sum(nh)/sum(np_));
srt = sort(allmiss);
fprintf('  miss distance at pulses : median %.2f m, 95th pct %.2f m, max %.2f m\n', ...
   median(allmiss), srt(max(1, ceil(0.95*numel(srt)))), max(allmiss));
figure(3); clf;
subplot(1,2,1); hist(tint, 20); grid on; xlabel('interception time [s]'); ylabel('cases'); %#ok<HIST>
subplot(1,2,2); hist(allmiss, 30); grid on; hold on; yl = ylim; plot(P.R_eng*[1 1], yl, 'r--'); %#ok<HIST>
xlabel('miss distance at pulse [m]'); ylabel('pulses');
save_fig('fig_mc');
diary off;
