function plot_info = plot_remote_guidance_window_contours(td_list, dt_list, dv_total_map, config)
% plot_remote_guidance_window_contours 绘制远程导引 Lambert 搜索窗口的自适应等高线图。
%
% 输入：
%   td_list        出发时刻搜索网格，单位为秒。
%   dt_list        飞行时间搜索网格，单位为秒。
%   dv_total_map   搜索得到的总脉冲二维分布。
%   config         等高线图配置结构体。
%
% 输出：
%   plot_info      图窗、颜色条与等高线信息结构体。

arguments
    td_list (1,:) double
    dt_list (1,:) double
    dv_total_map double
    config (1,1) struct {mustBeRemoteGuidanceContourPlotConfig} = struct( ...
        'dv_upper_limit', 1000, ...
        'level_count', 10, ...
        'contour_width', 1, ...
        'colorbar_tick_count_max', 6)
end

fig = utils.createFigureA4(struct( ...
    'Name', '远程导引 Lambert 窗口等高线图', ...
    'Width', 20, ...
    'AspectRatio', 0.65));
ax = axes('Parent', fig);

dv_display_map = dv_total_map;
dv_valid = dv_display_map(isfinite(dv_display_map));
[td_grid, dt_grid] = meshgrid(td_list / 3600, dt_list / 60);
has_contour_plot = false;

if ~isempty(dv_valid)
    contour_info = build_adaptive_contour_levels( ...
        dv_valid, config.dv_upper_limit, config.level_count);
    dv_contour_map = dv_display_map;
    dv_contour_map(dv_contour_map > contour_info.display_max) = nan;

    if contour_info.has_data
        contour(ax, td_grid, dt_grid, dv_contour_map, ...
            contour_info.levels, ...
            'LineWidth', config.contour_width);
        hold(ax, 'on');

        fprintf('等高线显示 Δv 上限 = %.4f m/s（按绝对阈值筛除更高区域）\n', ...
            contour_info.display_max);

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
grid(ax, 'on');grid(ax, 'minor');
hold(ax, 'on');
box(ax, 'on');
xlim(ax, [td_list(1), td_list(end)] / 3600);
ylim(ax, [dt_list(1), dt_list(end)] / 60);
utils.applyViridisColormap(256, false);

if has_contour_plot
    cb = colorbar(ax);
    [cb.Ticks, cb.TickLabels] = build_colorbar_ticks( ...
        contour_info.levels, config.colorbar_tick_count_max);
    cb.Label.String = utils.formatMixedFontText('总 \Delta{\itv} (m/s)');
else
    cb = [];
end

xlabel(ax, utils.formatMixedFontText('出发时刻 {\itt}_{\itd} (h)'));
ylabel(ax, utils.formatMixedFontText('飞行时间 \Delta{\itt} (min)'));
title(ax, utils.formatMixedFontText('二维共面远程导引 Lambert 窗口等高线图'));

plot_info = struct();
plot_info.fig = fig;
plot_info.ax = ax;
plot_info.colorbar = cb;
plot_info.contour_info = contour_info;
plot_info.has_contour_plot = has_contour_plot;

end

function mustBeRemoteGuidanceContourPlotConfig(config)
% mustBeRemoteGuidanceContourPlotConfig 校验等高线图配置结构体。

if ~isstruct(config) || ~isscalar(config)
    error('mustBeRemoteGuidanceContourPlotConfig:InvalidType', ...
        'config 必须为标量 struct。');
end

required_fields = {'dv_upper_limit', 'level_count', 'contour_width', ...
    'colorbar_tick_count_max'};
missing_fields = required_fields(~isfield(config, required_fields));
if ~isempty(missing_fields)
    error('mustBeRemoteGuidanceContourPlotConfig:MissingField', ...
        'config 缺少字段：%s', strjoin(missing_fields, ', '));
end

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

function contour_info = build_adaptive_contour_levels(data, dv_upper_limit, level_count_target)
% build_adaptive_contour_levels 生成带绝对上限的自适应等高线级别。

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

function [tick_values, tick_labels] = build_colorbar_ticks(levels, tick_count_max)
% build_colorbar_ticks 为颜色条抽样主刻度，避免标签重叠。

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
    @(value) format_tick_label(value, decimals), ...
    tick_values, ...
    'UniformOutput', false);

end

function tick_label = format_tick_label(value, decimals)
% format_tick_label 生成紧凑的颜色条刻度标签。

tick_label = sprintf(['%0.', num2str(decimals), 'f'], value);
tick_label = regexprep(tick_label, '(\.\d*?)0+$', '$1');
tick_label = regexprep(tick_label, '\.$', '');

end
