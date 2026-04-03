function setDefaultGraphics(english_font_name, font_size, chinese_font_name)
% SETDEFAULTGRAPHICS 统一设置全局绘图默认字体与字号

if nargin < 1 || isempty(english_font_name)
    english_font_name = 'Times New Roman';
end

if nargin < 2 || isempty(font_size)
    font_size = 10;
end

if nargin < 3 || isempty(chinese_font_name)
    chinese_font_name = 'SimSun';
end

setappdata(groot, 'DefaultEnglishFontName', english_font_name);
setappdata(groot, 'DefaultChineseFontName', chinese_font_name);

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
