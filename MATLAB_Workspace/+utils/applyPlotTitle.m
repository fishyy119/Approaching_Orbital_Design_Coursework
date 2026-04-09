function title_handle = applyPlotTitle(varargin)
% applyPlotTitle 按全局绘图模式应用坐标轴标题。

if nargin < 1
    error('utils.applyPlotTitle:NotEnoughInputs', ...
        '至少需要提供标题文本或坐标轴句柄。');
end

if isgraphics(varargin{1}, 'axes')
    ax = varargin{1};
    title_args = varargin(2:end);
else
    ax = gca;
    title_args = varargin;
end

if isempty(title_args)
    title_args = {''};
end

plot_mode = "presentation";
if isappdata(groot, 'PlotMode')
    plot_mode = lower(string(getappdata(groot, 'PlotMode')));
end

if plot_mode == "paper"
    title_handle = title(ax, '');
    title_handle.String = '';
    title_handle.Visible = 'off';
    return;
end

title_handle = title(ax, title_args{:});
title_handle.Visible = 'on';

end
