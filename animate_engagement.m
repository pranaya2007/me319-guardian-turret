function animate_engagement(scn, gifname)
%ANIMATE_ENGAGEMENT  Write a GIF of the engagement (target, beam, pulses).
if nargin < 1, scn = 'C'; end
if nargin < 2, gifname = ['scen' scn '.gif']; end
P = guardian_params();
o = simulate_engagement(scn);
idx = 1:50:numel(o.t);                    % one frame per 50 ms
fig = figure('Color','w','Position',[100 100 900 520]);
for n = 1:numel(idx)
    k = idx(n);  t = o.t(k);
    clf; hold on; grid on; axis equal;
    th = linspace(0, pi/2, 100);          % range envelope
    plot(800*cos(th), 800*sin(th), 'k:', 2000*cos(th), 2000*sin(th), 'k:');
    plot(o.x(1:k), o.y(1:k), 'b-');       % target path so far
    plot(o.x(k), o.y(k), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);
    flash = any(t - o.pulses(:,1) >= 0 & t - o.pulses(:,1) < 0.1);
    if flash, col = [0 .7 0]; lw = 3; else, col = [.85 0 0]; lw = 1; end
    plot([0 2200*cos(o.theta(k))], [0 2200*sin(o.theta(k))], '-', 'Color', col, 'LineWidth', lw);
    plot(0, 0, 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
    xlim([-100 2200]);  ylim([0 1400]);
    xlabel('x [m]'); ylabel('y [m]');
    title(sprintf('Scenario %s   t = %.2f s   miss = %.2f m   (%d pulses)', ...
        scn, t, o.miss(k), sum(o.pulses(:,1) <= t)));
    if ~o.valid(k) && t >= 1 && t < 1.5, text(100, 1300, 'RADAR LOSS', 'Color', 'r', 'FontSize', 14); end
    drawnow;
    [A, map] = rgb2ind(frame2im(getframe(fig)), 256);
    if n == 1
        imwrite(A, map, gifname, 'gif', 'LoopCount', Inf, 'DelayTime', 0.05);
    else
        imwrite(A, map, gifname, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
    end
end
end