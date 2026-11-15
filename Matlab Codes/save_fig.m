function save_fig(name)
%SAVE_FIG  Save the current figure as a vector PDF in ../paper/figures (MATLAB or Octave).
d = fullfile('..', 'paper', 'figures');
if ~exist(d, 'dir'), mkdir(d); end
file = fullfile(d, [name '.pdf']);
if exist('exportgraphics', 'file')
    exportgraphics(gcf, file, 'ContentType', 'vector');
else
    set(gcf, 'PaperPositionMode', 'auto');
    print(gcf, '-dpdf', file);
end
end
