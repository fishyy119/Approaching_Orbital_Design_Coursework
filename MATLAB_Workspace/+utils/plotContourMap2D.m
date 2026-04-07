function plot_info = plotContourMap2D(x_list, y_list, dv_total_map, config)
% plotContourMap2D 绘制仓库共用的二维自适应等高线图。
%
% 输入：
%   x_list        横轴参数网格。
%   y_list        纵轴参数网格。
%   dv_total_map  总脉冲二维分布。
%   config        绘图配置结构体。
%
% 输出：
%   plot_info     图窗、坐标轴、颜色条与等高线信息结构体。

arguments
    x_list (1,:) double
    y_list (1,:) double
    dv_total_map double
    config (1,1) struct {mustBeContourMapConfig}
end

if ~ismatrix(dv_total_map) ...
        || size(dv_total_map, 1) ~= numel(y_list) ...
        || size(dv_total_map, 2) ~= numel(x_list)
    error('utils.plotContourMap2D:InvalidMapSize', ...
        'dv_total_map 的尺寸必须为 numel(y_list) x numel(x_list)。');
end

fig = utils.createFigureA4(struct( ...
    'Name', char(config.figure_name), ...
    'Width', config.figure_width, ...
    'AspectRatio', config.aspect_ratio));
ax = axes('Parent', fig);

dv_display_map = dv_total_map;
dv_valid = dv_display_map(isfinite(dv_display_map));
[x_grid, y_grid] = meshgrid(x_list * config.x_scale, y_list * config.y_scale);
has_contour_plot = false;

if ~isempty(dv_valid)
    contour_info = buildAdaptiveContourLevels( ...
        dv_valid, config.dv_upper_limit, config.level_count);
    dv_contour_map = dv_display_map;
    dv_contour_map(dv_contour_map > contour_info.display_max) = nan;

    if contour_info.has_data
        contour(ax, x_grid, y_grid, dv_contour_map, ...
            contour_info.levels, ...
            'LineWidth', config.contour_width);
        hold(ax, 'on');

        if contour_info.display_max > contour_info.display_min
            clim(ax, [contour_info.display_min, contour_info.display_max]);
        end

        has_contour_plot = true;
    else
        warning('绝对上限 %.2f m/s 以下无可绘制的等高线数据。', config.dv_upper_limit);
    end
else
    contour_info = struct( ...
        'levels', nan, ...
        'display_min', nan, ...
        'display_max', nan, ...
        'cap_applied', false, ...
        'has_data', false);
    warning('等高线图无可用数据可绘制。');
end

set(ax, 'YDir', 'normal');
grid(ax, 'on');
grid(ax, 'minor');
box(ax, 'on');
xlim(ax, [x_grid(1, 1), x_grid(1, end)]);
ylim(ax, [y_grid(1, 1), y_grid(end, 1)]);
utils.applyViridisColormap(256, false);

if has_contour_plot
    cb = colorbar(ax);
    [cb.Ticks, cb.TickLabels] = buildColorbarTicks( ...
        contour_info.levels, config.colorbar_tick_count_max);
    cb.Label.String = utils.formatMixedFontText(char(config.colorbar_label));
else
    cb = [];
end

xlabel(ax, utils.formatMixedFontText(char(config.xlabel_text)));
ylabel(ax, utils.formatMixedFontText(char(config.ylabel_text)));
title(ax, utils.formatMixedFontText(char(config.title_text)));

plot_info = struct();
plot_info.fig = fig;
plot_info.ax = ax;
plot_info.colorbar = cb;
plot_info.contour_info = contour_info;
plot_info.has_contour_plot = has_contour_plot;

end

function mustBeContourMapConfig(config)
% mustBeContourMapConfig 校验共用等高线图配置结构体。

if ~isstruct(config) || ~isscalar(config)
    error('mustBeContourMapConfig:InvalidType', ...
        'config 必须为标量 struct。');
end

required_fields = {'figure_name', 'figure_width', 'aspect_ratio', ...
    'title_text', 'xlabel_text', 'ylabel_text', 'colorbar_label', ...
    'x_scale', 'y_scale', 'dv_upper_limit', 'level_count', ...
    'contour_width', 'colorbar_tick_count_max'};
missing_fields = required_fields(~isfield(config, required_fields));
if ~isempty(missing_fields)
    error('mustBeContourMapConfig:MissingField', ...
        'config 缺少字段：%s', strjoin(missing_fields, ', '));
end

mustBeTextScalarValue(config.figure_name, 'config.figure_name');
mustBeTextScalarValue(config.title_text, 'config.title_text');
mustBeTextScalarValue(config.xlabel_text, 'config.xlabel_text');
mustBeTextScalarValue(config.ylabel_text, 'config.ylabel_text');
mustBeTextScalarValue(config.colorbar_label, 'config.colorbar_label');

validateattributes(config.figure_width, {'double'}, ...
    {'real', 'finite', 'scalar', 'positive'}, ...
    mfilename, 'config.figure_width');
validateattributes(config.aspect_ratio, {'double'}, ...
    {'real', 'finite', 'scalar', 'positive'}, ...
    mfilename, 'config.aspect_ratio');
validateattributes(config.x_scale, {'double'}, ...
    {'real', 'finite', 'scalar', 'positive'}, ...
    mfilename, 'config.x_scale');
validateattributes(config.y_scale, {'double'}, ...
    {'real', 'finite', 'scalar', 'positive'}, ...
    mfilename, 'config.y_scale');
validateattributes(config.dv_upper_limit, {'double'}, ...
    {'real', 'finite', 'scalar', 'positive'}, ...
    mfilename, 'config.dv_upper_limit');
validateattributes(config.level_count, {'double'}, ...
    {'real', 'finite', 'scalar', 'positive'}, ...
    mfilename, 'config.level_count');
mustBeInteger(config.level_count);
validateattributes(config.contour_width, {'double'}, ...
    {'real', 'finite', 'scalar', 'positive'}, ...
    mfilename, 'config.contour_width');
validateattributes(config.colorbar_tick_count_max, {'double'}, ...
    {'real', 'finite', 'scalar', 'positive'}, ...
    mfilename, 'config.colorbar_tick_count_max');
mustBeInteger(config.colorbar_tick_count_max);

end

function mustBeTextScalarValue(value, value_name)
% mustBeTextScalarValue 校验文本字段为标量字符串或行字符向量。

is_valid_string = isstring(value) && isscalar(value);
is_valid_char = ischar(value) && (isrow(value) || isempty(value));

if ~(is_valid_string || is_valid_char)
    error('mustBeTextScalarValue:InvalidText', ...
        '%s 必须为标量字符串或行字符向量。', value_name);
end

end

function contour_info = buildAdaptiveContourLevels(data, dv_upper_limit, level_count_target)
% buildAdaptiveContourLevels 生成带绝对上限的自适应等高线级别。

data = sort(data(:));
data = data(isfinite(data));

if isempty(data)
    contour_info = struct( ...
        'levels', nan, ...
        'display_min', nan, ...
        'display_max', nan, ...
        'cap_applied', false, ...
        'has_data', false);
    return;
end

full_max = data(end);
data_in_range = data(data <= dv_upper_limit);

if isempty(data_in_range)
    contour_info = struct( ...
        'levels', nan, ...
        'display_min', nan, ...
        'display_max', min(full_max, dv_upper_limit), ...
        'cap_applied', dv_upper_limit < full_max, ...
        'has_data', false);
    return;
end

display_min = data_in_range(1);
display_max = data_in_range(end);
cap_applied = display_max < full_max;

if display_max <= display_min
    delta = max([1e-6, abs(display_min) * 1e-6, 1e-3]);
    levels = [display_min, display_min + delta];
else
    range_ratio = (display_max - display_min) / max(abs(display_min), 1);
    focus_power = min(3.0, max(1.6, 1.6 + 0.35 * log10(range_ratio + 1)));
    level_count = max(2, round(level_count_target));
    s = linspace(0, 1, level_count) .^ focus_power;
    levels = display_min + (display_max - display_min) * s;
    levels = unique(levels);

    if numel(levels) < 2
        levels = [display_min, display_max];
    end
end

contour_info = struct( ...
    'levels', levels, ...
    'display_min', display_min, ...
    'display_max', display_max, ...
    'cap_applied', cap_applied, ...
    'has_data', true);

end

function [tick_values, tick_labels] = buildColorbarTicks(levels, tick_count_max)
% buildColorbarTicks 为颜色条抽样主刻度，避免标签重叠。

tick_values = unique(levels(:).');
tick_values = tick_values(isfinite(tick_values));

if isempty(tick_values)
    tick_labels = {};
    return;
end

tick_count_max = max(2, round(tick_count_max));

if numel(tick_values) > tick_count_max
    tick_index = unique(round(linspace(1, numel(tick_values), tick_count_max)));
    tick_values = tick_values(tick_index);
end

if numel(tick_values) >= 2
    tick_step = min(diff(tick_values));
else
    tick_step = max(abs(tick_values(1)), 1);
end

if tick_step >= 10
    decimals = 0;
elseif tick_step >= 1
    decimals = 1;
elseif tick_step >= 0.1
    decimals = 2;
else
    decimals = 3;
end

tick_labels = arrayfun( ...
    @(value) formatTickLabel(value, decimals), ...
    tick_values, ...
    'UniformOutput', false);

end

function tick_label = formatTickLabel(value, decimals)
% formatTickLabel 生成紧凑的颜色条刻度标签。

tick_label = sprintf(['%0.', num2str(decimals), 'f'], value);
tick_label = regexprep(tick_label, '(\.\d*?)0+$', '$1');
tick_label = regexprep(tick_label, '\.$', '');

end
