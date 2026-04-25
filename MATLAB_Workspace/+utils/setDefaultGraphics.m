function setDefaultGraphics(english_font_name, font_size, chinese_font_name, plot_mode)
% SETDEFAULTGRAPHICS 统一设置全局绘图默认字体、字号与绘图模式

if nargin < 1 || isempty(english_font_name)
    english_font_name = 'Times New Roman';
end

if nargin < 2 || isempty(font_size)
    font_size = 9;
end

if nargin < 3 || isempty(chinese_font_name)
    chinese_font_name = 'SimSun';
end

if nargin < 4 || isempty(plot_mode)
    % 全局绘图模式开关：
    % "presentation" 显示图内标题，适合展示场景；
    % "paper" 隐藏图内标题，适合论文插图场景。
    plot_mode = "paper";
    % plot_mode = "presentation";
end

plot_mode = normalize_plot_mode(plot_mode);

setappdata(groot, 'DefaultEnglishFontName', english_font_name);
setappdata(groot, 'DefaultChineseFontName', chinese_font_name);
setappdata(groot, 'PlotMode', plot_mode);

set(groot, ...
    'DefaultAxesFontName', english_font_name, ...
    'DefaultTextFontName', english_font_name, ...
    'DefaultAxesFontSize', font_size, ...
    'DefaultTextFontSize', font_size, ...
    'DefaultLineLineWidth', 1.2, ...
    'DefaultTextInterpreter', 'tex', ...
    'DefaultLegendInterpreter', 'tex', ...
    'DefaultAxesTickLabelInterpreter', 'tex');

end

function plot_mode = normalize_plot_mode(plot_mode)
% normalize_plot_mode 规范化绘图模式字符串。

plot_mode = lower(string(plot_mode));

if ~ismember(plot_mode, ["presentation", "paper"])
    error('utils.setDefaultGraphics:InvalidPlotMode', ...
        'plot_mode 仅支持 "presentation" 或 "paper"。');
end

end
