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
    config (1,1) struct = struct()
end

shared_config = struct();
shared_config.figure_name = '远程导引 Lambert 窗口等高线图';
shared_config.figure_width = 20;
shared_config.aspect_ratio = 0.68;
shared_config.title_text = '二维共面远程导引 Lambert 窗口等高线图';
shared_config.xlabel_text = '出发时刻 {\itt}_{\itd} (h)';
shared_config.ylabel_text = '飞行时间 \Delta{\itt} (min)';
shared_config.colorbar_label = '总 \Delta{\itv} (m/s)';
shared_config.x_scale = 1 / 3600;
shared_config.y_scale = 1 / 60;
shared_config.dv_upper_limit = getConfigValue(config, 'dv_upper_limit', 1000);
shared_config.level_count = getConfigValue(config, 'level_count', 10);
shared_config.contour_width = getConfigValue(config, 'contour_width', 1);
shared_config.colorbar_tick_count_max = getConfigValue(config, ...
    'colorbar_tick_count_max', 6);

plot_info = utils.plotContourMap2D(td_list, dt_list, dv_total_map, shared_config);

if plot_info.has_contour_plot
    fprintf('等高线显示 Δv 上限 = %.4f m/s（按绝对阈值筛除更高区域）\n', ...
        plot_info.contour_info.display_max);
end

end

function value = getConfigValue(config, field_name, default_value)
% getConfigValue 读取配置字段，若缺失则回退到默认值。

if isfield(config, field_name)
    value = config.(field_name);
else
    value = default_value;
end

end
