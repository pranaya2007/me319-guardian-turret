function make_frames()
% MAKE_FRAMES  Four snapshots of scenario C -> fig_frames.pdf
o = simulate_engagement('C');
tt = [0.6 1.3 2.4 5.0];
th = linspace(0, pi/2, 100);
figure('Color', 'w', 'Position', [50 50 1400 330]);
for j = 1:4
    k = round(tt(j)/1e-3) + 1;
    t = o.t(k);
    subplot(1, 4, j); hold on; grid on; axis equal;
    plot(800*cos(th), 800*sin(th), 'k:');
    plot(2000*cos(th), 2000*sin(th), 'k:');
    plot(o.x(1:k), o.y(1:k), 'b-');
    plot(o.x(k), o.y(k), 'bo', 'MarkerFaceColor', 'b');
    dtp = t - o.pulses(:,1);
    if any(dtp >= 0 & dtp < 0.15)
        plot([0 2200*cos(o.theta(k))], [0 2200*sin(o.theta(k))], 'r-', 'LineWidth', 2.5);
    else
        plot([0 2200*cos(o.theta(k))], [0 2200*sin(o.theta(k))], 'r--');
    end
    plot(0, 0, 'ks', 'MarkerFaceColor', 'k');
    xlim([-100 2200]); ylim([0 1400]);
    xlabel('x [m]'); if j == 1, ylabel('y [m]'); end
    title(sprintf('t = %.1f s', t));
end
save_fig('fig_frames');
end