function plot_info = plot_proximity_search_contours_2d(x_list, y_list, dv_total_map, config)
% plot_proximity_search_contours_2d 绘制近距离导引参数归并等高线图。
%
% 输入：
%   x_list        横轴参数网格。
%   y_list        纵轴参数网格。
%   dv_total_map  对其余维度归并后的总脉冲二维分布。
%   config        等高线图配置结构体。
%
% 输出：
%   plot_info      图窗、颜色条与等高线信息结构体。

arguments
    x_list (1,:) double
    y_list (1,:) double
    dv_total_map double
    config (1,1) struct = struct()
end

shared_config = struct();
shared_config.figure_name = getConfigValue(config, 'figure_name', ...
    '近距离导引参数归并等高线图');
shared_config.figure_width = getConfigValue(config, 'figure_width', 20);
shared_config.aspect_ratio = getConfigValue(config, 'aspect_ratio', 0.68);
shared_config.title_text = getConfigValue(config, 'title_text', ...
    '近距离导引参数归并等高线图');
shared_config.xlabel_text = getConfigValue(config, 'xlabel_text', '参数 x');
shared_config.ylabel_text = getConfigValue(config, 'ylabel_text', '参数 y');
shared_config.colorbar_label = getConfigValue(config, 'colorbar_label', ...
    '总 \Delta{\itv} (m/s)');
shared_config.x_scale = getConfigValue(config, 'x_scale', 1);
shared_config.y_scale = getConfigValue(config, 'y_scale', 1);
shared_config.dv_upper_limit = getConfigValue(config, 'dv_upper_limit', 100);
shared_config.level_count = getConfigValue(config, 'level_count', 10);
shared_config.contour_width = getConfigValue(config, 'contour_width', 1);
shared_config.colorbar_tick_count_max = getConfigValue(config, ...
    'colorbar_tick_count_max', 6);

plot_info = utils.plotContourMap2D(x_list, y_list, dv_total_map, shared_config);

end

function value = getConfigValue(config, field_name, default_value)
% getConfigValue 读取配置字段，若缺失则回退到默认值。

if isfield(config, field_name)
    value = config.(field_name);
else
    value = default_value;
end

end
