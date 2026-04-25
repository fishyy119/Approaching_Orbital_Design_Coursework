function plot_info = plotContourMap2D(x_list, y_list, value_map, config)
% plotContourMap2D 绘制仓库共用的二维自适应等高线图。
%
% 输入：
%   x_list        横轴参数网格。
%   y_list        纵轴参数网格。
%   value_map     待展示的二维标量分布。
%   config        绘图配置结构体。
%
% 输出：
%   plot_info     图窗、坐标轴、颜色条与等高线信息结构体。

arguments
    x_list (1,:) double
    y_list (1,:) double
    value_map double
    config (1,1) struct {mustBeContourMapConfig}
end

if ~ismatrix(value_map) ...
        || size(value_map, 1) ~= numel(y_list) ...
        || size(value_map, 2) ~= numel(x_list)
    error('utils.plotContourMap2D:InvalidMapSize', ...
        'value_map 的尺寸必须为 numel(y_list) x numel(x_list)。');
end

fig = utils.createFigureA4(struct( ...
    'Name', char(config.figure_name), ...
    'Width', config.figure_width, ...
    'AspectRatio', config.aspect_ratio));
ax = axes('Parent', fig);

contour_map = value_map;
valid_data = contour_map(isfinite(contour_map));
[x_grid, y_grid] = meshgrid(x_list * config.x_scale, y_list * config.y_scale);
has_contour_plot = false;

if ~isempty(valid_data)
    contour_info = buildAdaptiveContourLevels( ...
        valid_data, config.display_upper_limit, config.level_count);
    contour_map(contour_map > contour_info.display_max) = nan;

    if contour_info.has_data
        contour(ax, x_grid, y_grid, contour_map, ...
            contour_info.levels, ...
            'LineWidth', config.contour_width);
        hold(ax, 'on');

        if contour_info.display_max > contour_info.display_min
            clim(ax, [contour_info.display_min, contour_info.display_max]);
        end

        has_contour_plot = true;
    else
        warning('绝对显示上限 %.4f 以下无可绘制的等高线数据。', ...
            config.display_upper_limit);
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
% grid(ax, 'minor');
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
utils.applyPlotTitle(ax, utils.formatMixedFontText(char(config.title_text)));

plot_info = struct();
plot_info.fig = fig;
plot_info.ax = ax;
plot_info.colorbar = cb;
plot_info.contour_info = contour_info;
plot_info.has_contour_plot = has_contour_plot;

end

function mustBeContourMapConfig(config)
% mustBeContourMapConfig 校验共用等高线图配置结构体。

schema = { ...
    'figure_name', utils.schema.textScalar(); ...
    'figure_width', utils.schema.doubleScalar('positive'); ...
    'aspect_ratio', utils.schema.doubleScalar('positive'); ...
    'title_text', utils.schema.textScalar(); ...
    'xlabel_text', utils.schema.textScalar(); ...
    'ylabel_text', utils.schema.textScalar(); ...
    'colorbar_label', utils.schema.textScalar(); ...
    'x_scale', utils.schema.doubleScalar('positive'); ...
    'y_scale', utils.schema.doubleScalar('positive'); ...
    'display_upper_limit', utils.schema.doubleScalar('positive'); ...
    'level_count', utils.schema.doubleScalar('integer', 'positive'); ...
    'contour_width', utils.schema.doubleScalar('positive'); ...
    'colorbar_tick_count_max', utils.schema.doubleScalar('integer', 'positive')};

utils.schema.validateStruct(config, schema, 'config');

end

function contour_info = buildAdaptiveContourLevels(data, display_upper_limit, level_count_target)
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
data_in_range = data(data <= display_upper_limit);

if isempty(data_in_range)
    contour_info = struct( ...
        'levels', nan, ...
        'display_min', nan, ...
        'display_max', min(full_max, display_upper_limit), ...
        'cap_applied', display_upper_limit < full_max, ...
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
