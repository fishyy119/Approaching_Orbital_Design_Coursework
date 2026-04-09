function fig = plot_proximity_optimal_dv_vs_time_by_N_2d(summary, config)
% plot_proximity_optimal_dv_vs_time_by_N_2d 绘制不同 N 下转移时间-最佳总 Δv 折线图。
%
% 输入：
%   summary   各 N 下转移时间-最佳总 Δv 前沿结构体。
%   config    折线图配置结构体。
%
% 输出：
%   fig       图窗句柄。

arguments
    summary (1,1) struct {mustBeDvTimeSummary}
    config (1,1) struct {mustBeDvTimePlotConfig}
end

fig = utils.createFigureA4(struct( ...
    'Name', char(config.figure_name), ...
    'Width', config.figure_width, ...
    'AspectRatio', config.aspect_ratio));
ax = axes('Parent', fig);
hold(ax, 'on');

valid_line_count = 0;
for i_N = 1:numel(summary.N_list)
    time_list = summary.transfer_time_list_by_N{i_N};
    dv_list = summary.dv_total_list_by_N{i_N};

    if isempty(time_list) || isempty(dv_list)
        continue;
    end

    valid_line_count = valid_line_count + 1;
    plot(ax, time_list * config.x_scale, dv_list, '-', ...
        'LineWidth', config.line_width, ...
        'DisplayName', sprintf('{\\itN} = %d', summary.N_list(i_N)));
end

grid(ax, 'on');
grid(ax, 'minor');
box(ax, 'on');

if valid_line_count == 0
    warning('plot_proximity_optimal_dv_vs_time_by_N_2d:NoValidLine', ...
        '当前 summary 中没有可绘制的折线数据。');
end

xlabel(ax, utils.formatMixedFontText(char(config.xlabel_text)));
ylabel(ax, utils.formatMixedFontText(char(config.ylabel_text)));
utils.applyPlotTitle(ax, utils.formatMixedFontText(char(config.title_text)));
legend(ax, 'Location', 'best', 'Interpreter', 'tex');

end

function mustBeDvTimeSummary(summary)
% mustBeDvTimeSummary 校验转移时间-最佳总 Δv 折线图摘要结构体。

schema = { ...
    'N_list', utils.schema.doubleVector('nonempty'); ...
    'transfer_time_list_by_N', utils.schema.cellVector(); ...
    'dv_total_list_by_N', utils.schema.cellVector()};

utils.schema.validateStruct(summary, schema, 'summary');

if numel(summary.N_list) ~= numel(summary.transfer_time_list_by_N) ...
        || numel(summary.N_list) ~= numel(summary.dv_total_list_by_N)
    error('mustBeDvTimeSummary:InconsistentSize', ...
        'summary 中 N_list 与各 cell 列表长度必须一致。');
end

for i = 1:numel(summary.N_list)
    validateattributes(summary.transfer_time_list_by_N{i}, {'double'}, ...
        {'real', 'vector'}, mfilename, 'summary.transfer_time_list_by_N{i}');
    validateattributes(summary.dv_total_list_by_N{i}, {'double'}, ...
        {'real', 'vector'}, mfilename, 'summary.dv_total_list_by_N{i}');

    if numel(summary.transfer_time_list_by_N{i}) ~= numel(summary.dv_total_list_by_N{i})
        error('mustBeDvTimeSummary:InconsistentPointCount', ...
            '第 %d 条折线的横纵坐标点数不一致。', i);
    end
end

end

function mustBeDvTimePlotConfig(config)
% mustBeDvTimePlotConfig 校验转移时间-最佳总 Δv 折线图配置。

schema = { ...
    'figure_name', utils.schema.textScalar(); ...
    'figure_width', utils.schema.doubleScalar('positive'); ...
    'aspect_ratio', utils.schema.doubleScalar('positive'); ...
    'title_text', utils.schema.textScalar(); ...
    'xlabel_text', utils.schema.textScalar(); ...
    'ylabel_text', utils.schema.textScalar(); ...
    'x_scale', utils.schema.doubleScalar('positive'); ...
    'line_width', utils.schema.doubleScalar('positive')};

utils.schema.validateStruct(config, schema, 'config');

end
