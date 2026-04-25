function fig = plot_proximity_optimal_dv_vs_N_2d(N_list, metrics, config)
% plot_proximity_optimal_dv_vs_N_2d 绘制各 N 最优速度增量指标折线图。
%
% 输入：
%   N_list    脉冲段数列表。
%   metrics   指标结构体，包含总 Δv、单次脉冲最大值与最小值。
%   config    折线图配置结构体。
%
% 输出：
%   fig       图窗句柄。

arguments
    N_list (1,:) double
    metrics (1,1) struct {mustBeMetricConfig}
    config (1,1) struct {mustBeOptimalDvPlotConfig}
end

fig = utils.createFigureA4(struct( ...
    'Name', char(config.figure_name), ...
    'Width', config.figure_width, ...
    'AspectRatio', config.aspect_ratio));
ax = axes('Parent', fig);
hold(ax, 'on');

h_total = plot(ax, N_list, metrics.dv_total_list, '-o', ...
    'LineWidth', config.line_width, ...
    'MarkerSize', config.marker_size);
h_single_max = plot(ax, N_list, metrics.dv_single_max_list, '-s', ...
    'LineWidth', config.line_width, ...
    'MarkerSize', config.marker_size);
h_single_min = plot(ax, N_list, metrics.dv_single_min_list, '-d', ...
    'LineWidth', config.line_width, ...
    'MarkerSize', config.marker_size);

grid(ax, 'on');
% grid(ax, 'minor');
box(ax, 'on');

if numel(unique(N_list)) == 1
    xlim(ax, N_list(1) + [-0.5, 0.5]);
else
    xlim(ax, [min(N_list), max(N_list)]);
end

if all(isfinite(N_list)) && numel(N_list) <= 20
    xticks(ax, N_list);
end

xlabel(ax, utils.formatMixedFontText(char(config.xlabel_text)));
ylabel(ax, utils.formatMixedFontText(char(config.ylabel_text)));
utils.applyPlotTitle(ax, utils.formatMixedFontText(char(config.title_text)));

legend_labels = utils.formatMixedFontText({ ...
    '总 \Delta{\itv}', ...
    '单次脉冲最大值', ...
    '单次脉冲最小值'});

delete(findall(fig, 'Type', 'Legend'));
legend_handle = legend(ax, [h_total, h_single_max, h_single_min], ...
    legend_labels, ...
    'Location', 'best', ...
    'Interpreter', 'tex');
legend_handle.AutoUpdate = 'off';

end

function mustBeMetricConfig(metrics)
% mustBeMetricConfig 校验折线图指标结构体。

schema = { ...
    'dv_total_list', utils.schema.doubleVector(); ...
    'dv_single_max_list', utils.schema.doubleVector(); ...
    'dv_single_min_list', utils.schema.doubleVector()};

utils.schema.validateStruct(metrics, schema, 'metrics');

metric_lengths = [ ...
    numel(metrics.dv_total_list), ...
    numel(metrics.dv_single_max_list), ...
    numel(metrics.dv_single_min_list)];

if numel(unique(metric_lengths)) ~= 1
    error('mustBeMetricConfig:InconsistentSize', ...
        'metrics 中各指标列表长度必须一致。');
end

end

function mustBeOptimalDvPlotConfig(config)
% mustBeOptimalDvPlotConfig 校验最优速度增量折线图配置。

schema = { ...
    'figure_name', utils.schema.textScalar(); ...
    'figure_width', utils.schema.doubleScalar('positive'); ...
    'aspect_ratio', utils.schema.doubleScalar('positive'); ...
    'title_text', utils.schema.textScalar(); ...
    'xlabel_text', utils.schema.textScalar(); ...
    'ylabel_text', utils.schema.textScalar(); ...
    'line_width', utils.schema.doubleScalar('positive'); ...
    'marker_size', utils.schema.doubleScalar('positive')};

utils.schema.validateStruct(config, schema, 'config');

end
