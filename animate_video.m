function animate_video(scn, vidname, slow)
% ANIMATE_VIDEO  MP4 of the engagement, light background, time colorbar.
%   slow = slow-motion factor (default 4 -> 10 s of action lasts 40 s).
if nargin < 1, scn = 'C'; end
if nargin < 2, vidname = ['scen' scn '.mp4']; end
if nargin < 3, slow = 4; end

o = simulate_engagement(scn);
idx = 1:25:numel(o.t);

vw = VideoWriter(vidname, 'MPEG-4');
vw.FrameRate = 40/slow;
vw.Quality = 95;
open(vw);

fig = figure('Color', 'w', 'Position', [100 100 1000 560]);
try, fig.Theme = 'light'; catch, end
th = linspace(0, pi/2, 100);

for n = 1:numel(idx)
    k = idx(n);
    t = o.t(k);

    clf(fig);
    ax = axes('Parent', fig);
    hold(ax, 'on');
    set(ax, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
        'GridColor', [0.5 0.5 0.5], 'GridAlpha', 0.3);
    grid(ax, 'on');
    axis(ax, 'equal');
    colormap(ax, parula);
    clim(ax, [0 10]);

    plot(ax, 800*cos(th), 800*sin(th), 'k:');
    plot(ax, 2000*cos(th), 2000*sin(th), 'k:');

    ii = idx(1:n);
    scatter(ax, o.x(ii), o.y(ii), 14, o.t(ii), 'filled');
    plot(ax, o.x(k), o.y(k), 'o', 'MarkerFaceColor', [0 0.2 1], ...
        'MarkerEdgeColor', 'k', 'MarkerSize', 10);

    dtp = t - o.pulses(:,1);
    flash = any(dtp >= 0 & dtp < 0.1);
    if flash
        col = [0 0.7 0];
        lw = 3;
    else
        col = [0.85 0 0];
        lw = 1;
    end
    plot(ax, [0 2200*cos(o.theta(k))], [0 2200*sin(o.theta(k))], '-', ...
        'Color', col, 'LineWidth', lw);
    plot(ax, 0, 0, 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 8);

    xlim(ax, [-100 2200]);
    ylim(ax, [0 1400]);
    xlabel(ax, 'x [m]', 'Color', 'k');
    ylabel(ax, 'y [m]', 'Color', 'k');

    npl = sum(o.pulses(:,1) <= t);
    ttl = sprintf('Scenario %s, t = %.2f s, miss = %.2f m, pulses = %d', ...
        scn, t, o.miss(k), npl);
    title(ax, ttl, 'Color', 'k');

    cb = colorbar(ax);
    cb.Color = 'k';
    cb.Label.String = 'time [s]';
    cb.Label.Color = 'k';

    if strcmpi(scn, 'C') && t >= 1 && t < 1.5
        text(ax, 100, 1300, 'RADAR LOSS', 'Color', 'r', 'FontSize', 14);
    end

    drawnow;
    writeVideo(vw, getframe(fig));
end

close(vw);
end