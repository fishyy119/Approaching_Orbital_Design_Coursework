%% 近距离导引段：多脉冲指数趋近切入绕飞轨道
%
% 本脚本在二维共面条件下，对脉冲段数、指数趋近律调度导数与
% 第三四象限切入相位做四维参数搜索，并使用基于 CW 方程的
% 直线指数趋近律完成多脉冲抵近设计。
%
% 输出：
%   1. 起点调度导数-终点调度导数归并总 Δv 等高线图
%   2. 起点调度导数-终点调度导数归并最优切入相位颜色图
%   3. 各脉冲段数 N 对应最优速度增量指标折线图
%   4. 不同脉冲段数下转移时间-最佳总 Δv 折线图
%   5. 全局最优参数对应的相对系转移与切入后绕飞轨迹图
%   6. 惯性系 ode113 数值递推回代到 LVLH 系后的误差统计与
%      切入后绕飞保持误差

close all;
clear;
clc;
utils.setDefaultGraphics();

%% 常数与目标圆轨道
mu = 3.986004418e14; % m^3/s^2
Re = 6378137; % m
deg = pi / 180;

target = struct();
target.radius = Re + 1000e3;
target.theta0 = 80 * deg;
target.t0 = 0;
target.n = sqrt(mu / target.radius^3);
target.T = 2 * pi / target.n;

%% 初始捕获点状态
initial_relative = struct();
initial_relative.rel_r0 = [0; -5e3];
initial_relative.rel_v0 = [0; 0];

%% 绕飞轨道参数
orbit = struct();
orbit.k = 1e3;
orbit.yc = 0;

%% 多脉冲趋近与切入相位搜索参数
search_config = struct();
search_config.N_list = 5:5;
search_config.drho0_list = utils.buildGradedList( ...
    [-12, -8, -4, -1.5], ...
    [8, 8, 8]);
search_config.drhoT_list = utils.buildGradedList( ...
    [-1.5, -0.6, -0.2, -0.01], ...
    [8, 8, 8]);

% 最优相位几乎不落在第四象限。
search_config.phase_bounds = [pi, 3 * pi / 2];
search_config.phase_count = 91;
search_config.segment_samples = 80;
search_config.T_total_max = 40 * 60;

%% 切入相位搜索与最优多脉冲设计
search_result = proximity_search_multimpulse_plan_2d( ...
    target.n, initial_relative.rel_r0, initial_relative.rel_v0, orbit, search_config);
best_plan = search_result.best_plan;

%% 惯性系数值积分验证
simulation_config = struct();
simulation_config.segment_step = 10;
simulation_config.post_insert_duration = target.T / 2;
simulation_config.post_insert_step = 10;

simulation = simulate_impulsive_proximity_guidance_2d( ...
    mu, Re, target, initial_relative, best_plan, simulation_config);

%% 数据输出
disp("==== 近距离导引设计参数 ====");
fprintf('目标圆轨道半径 = %.3f km\n', target.radius / 1e3);
fprintf('目标轨道周期 = %.2f s\n', target.T);
fprintf('初始相对位置向量 = [%.3f %.3f] km\n', initial_relative.rel_r0 / 1e3);
fprintf('初始相对速度向量 = [%.4f %.4f] m/s\n', initial_relative.rel_v0);
fprintf('绕飞轨道参数 k = %.3f km, yc = %.3f km\n', orbit.k / 1e3, orbit.yc / 1e3);
fprintf('切入相位搜索区间（第三象限） = [%.1f, %.1f] deg\n', ...
    search_config.phase_bounds(1) * 180 / pi, search_config.phase_bounds(2) * 180 / pi);
fprintf('切入相位离散点数 = %d\n', search_config.phase_count);
fprintf('脉冲段数搜索集合 = [%s]\n', num2str(search_config.N_list));
fprintf('起点调度导数搜索集合 drho0 = [%s] m/s\n', num2str(search_config.drho0_list));
fprintf('终点调度导数搜索集合 drhoT = [%s] m/s\n', num2str(search_config.drhoT_list));
fprintf('总转移时间上限 = %.2f s (%.2f min)\n', ...
    search_config.T_total_max, search_config.T_total_max / 60);
fprintf('切入后绕飞保持验证时长 = %.2f s (%.2f min)\n', ...
    simulation_config.post_insert_duration, simulation_config.post_insert_duration / 60);
fprintf('总网格点数 = %d\n', search_result.total_case_count);
fprintf('满足导数顺序约束的网格点数 = %d\n', search_result.valid_schedule_case_count);
fprintf('求解成功的可行网格点数 = %d\n', search_result.feasible_case_count);
fprintf('因总转移时间超限被剔除的网格点数 = %d\n', search_result.time_limit_reject_count);

disp(" ");
disp("==== 最优切入结果 ====");
fprintf('最优脉冲段数 N = %d\n', best_plan.N);
fprintf('最优起点调度导数 drho0 = %.3f m/s\n', best_plan.drho0);
fprintf('最优终点调度导数 drhoT = %.3f m/s\n', best_plan.drhoT);
fprintf('最优切入相位 theta_in = %.6f rad (%.2f deg)\n', ...
    best_plan.theta, best_plan.theta * 180 / pi);
fprintf('切入点位置 = [%.3f %.3f] km\n', best_plan.rel_rf / 1e3);
fprintf('切入点速度 = [%.4f %.4f] m/s\n', best_plan.rel_vf);
fprintf('总转移时间 = %.2f s (%.2f min)\n', ...
    best_plan.T_total, best_plan.T_total / 60);
fprintf('段间隔 dt = %.2f s\n', best_plan.dt_segment);

for k = 1:best_plan.N
    fprintf('第 %d 段段首脉冲 Δv_%d = [%.4f %.4f] m/s, |Δv_%d| = %.4f m/s\n', ...
        k, k, best_plan.dv_segments(:, k), k, norm(best_plan.dv_segments(:, k)));
end

fprintf('末端切入脉冲 Δv_ins = [%.4f %.4f] m/s, |Δv_ins| = %.4f m/s\n', ...
    best_plan.dv_terminal, norm(best_plan.dv_terminal));
fprintf('总脉冲 ΔV = %.4f m/s\n', best_plan.dv_total);

disp(" ");
disp("==== LVLH 误差校核 ====");
fprintf('转移段最大位置误差 = %.6e m\n', simulation.validation.transfer_max_position_error);
fprintf('转移段最大速度误差 = %.6e m/s\n', simulation.validation.transfer_max_velocity_error);
fprintf('末端脉冲前位置误差 = %.6e m\n', simulation.validation.arrival_position_error);
fprintf('末端脉冲前速度误差 = %.6e m/s\n', simulation.validation.arrival_velocity_error);
fprintf('切入后绕飞最大位置误差 = %.6e m\n', ...
    simulation.validation.post_insert_max_position_error);
fprintf('切入后绕飞最大速度误差 = %.6e m/s\n', ...
    simulation.validation.post_insert_max_velocity_error);

%% 图 1：起止调度导数归并总 Δv 等高线图
contour_config_drho_dv = struct();
contour_config_drho_dv.figure_name = '近距离导引起止调度导数归并总Δv等高线图';
contour_config_drho_dv.figure_width = 20;
contour_config_drho_dv.aspect_ratio = 0.68;
contour_config_drho_dv.title_text = '起点调度导数-终点调度导数归并总 \Delta{\itv} 等高线图';
contour_config_drho_dv.xlabel_text = '起点调度导数 drho0 (m/s)';
contour_config_drho_dv.ylabel_text = '终点调度导数 drhoT (m/s)';
contour_config_drho_dv.colorbar_label = '总 \Delta{\itv} (m/s)';
contour_config_drho_dv.x_scale = 1;
contour_config_drho_dv.y_scale = 1;
contour_config_drho_dv.display_upper_limit = 100;
contour_config_drho_dv.level_count = 10;
contour_config_drho_dv.contour_width = 1;
contour_config_drho_dv.colorbar_tick_count_max = 6;

plot_drho_dv = utils.plotContourMap2D( ...
    search_result.projections.drho0_drhoT.x_list, ...
    search_result.projections.drho0_drhoT.y_list, ...
    search_result.projections.drho0_drhoT.dv_map, ...
    contour_config_drho_dv);

%% 图 2：起止调度导数归并最优切入相位颜色图
colormap_config_drho_theta = struct();
colormap_config_drho_theta.figure_name = '近距离导引起止调度导数归并最优切入相位颜色图';
colormap_config_drho_theta.figure_width = 20;
colormap_config_drho_theta.aspect_ratio = 0.68;
colormap_config_drho_theta.title_text = '起点调度导数-终点调度导数归并最优切入相位 \theta_{in} 颜色图';
colormap_config_drho_theta.xlabel_text = '起点调度导数 drho0 (m/s)';
colormap_config_drho_theta.ylabel_text = '终点调度导数 drhoT (m/s)';
colormap_config_drho_theta.colorbar_label = '最优切入相位 \theta_{in} (deg)';
colormap_config_drho_theta.x_scale = 1;
colormap_config_drho_theta.y_scale = 1;
colormap_config_drho_theta.display_upper_limit = search_config.phase_bounds(2) * 180 / pi;
colormap_config_drho_theta.level_count = 10;
colormap_config_drho_theta.colorbar_tick_count_max = 6;

plot_drho_theta = utils.plotColorMap2D( ...
    search_result.projections.drho0_drhoT.x_list, ...
    search_result.projections.drho0_drhoT.y_list, ...
    search_result.projections.drho0_drhoT.theta_deg_map, ...
    colormap_config_drho_theta);

%% 图 3：最优速度增量-脉冲段数关系图
lineplot_config_optimal_dv = struct();
lineplot_config_optimal_dv.figure_name = '近距离导引最优速度增量-脉冲段数关系图';
lineplot_config_optimal_dv.figure_width = 20;
lineplot_config_optimal_dv.aspect_ratio = 0.68;
lineplot_config_optimal_dv.title_text = '最优速度增量指标随脉冲段数 {\itN} 的变化';
lineplot_config_optimal_dv.xlabel_text = '脉冲段数 {\itN}';
lineplot_config_optimal_dv.ylabel_text = '速度增量 (m/s)';
lineplot_config_optimal_dv.line_width = 1.5;
lineplot_config_optimal_dv.marker_size = 7;

fig_dv_vs_N = plot_proximity_optimal_dv_vs_N_2d( ...
    search_result.optimal_by_N.N_list, ...
    search_result.optimal_by_N, ...
    lineplot_config_optimal_dv);

%% 图 4：转移时间-最佳总 Δv 关系图
lineplot_config_dv_time_by_N = struct();
lineplot_config_dv_time_by_N.figure_name = '近距离导引转移时间-最佳总速度增量关系图';
lineplot_config_dv_time_by_N.figure_width = 20;
lineplot_config_dv_time_by_N.aspect_ratio = 0.68;
lineplot_config_dv_time_by_N.title_text = '不同脉冲段数下转移时间-最佳总 \Delta{\itv} 关系图';
lineplot_config_dv_time_by_N.xlabel_text = '转移时间 {\itT} (min)';
lineplot_config_dv_time_by_N.ylabel_text = '最佳总 \Delta{\itv} (m/s)';
lineplot_config_dv_time_by_N.x_scale = 1 / 60;
lineplot_config_dv_time_by_N.line_width = 1.5;

fig_dv_time_by_N = plot_proximity_optimal_dv_vs_time_by_N_2d( ...
    search_result.optimal_dv_vs_time_by_N, ...
    lineplot_config_dv_time_by_N);

%% 图 5：相对系转移与切入后绕飞轨迹图
fig_transfer = plot_proximity_guidance_trajectory_2d(simulation);

%% 汇总输出结构体
proximity_result = struct();
proximity_result.mu = mu;
proximity_result.Re = Re;
proximity_result.target = target;
proximity_result.initial_relative = initial_relative;
proximity_result.orbit = orbit;
proximity_result.search_config = search_config;
proximity_result.search_result = search_result;
proximity_result.best_plan = best_plan;
proximity_result.simulation = simulation;
proximity_result.simulation_config = simulation_config;
proximity_result.figures = struct( ...
    'drho0_drhoT', plot_drho_dv.fig, ...
    'drho0_drhoT_theta', plot_drho_theta.fig, ...
    'optimal_dv_vs_N', fig_dv_vs_N, ...
    'optimal_dv_vs_time_by_N', fig_dv_time_by_N, ...
    'relative_mission', fig_transfer, ...
    'relative_transfer', fig_transfer);
