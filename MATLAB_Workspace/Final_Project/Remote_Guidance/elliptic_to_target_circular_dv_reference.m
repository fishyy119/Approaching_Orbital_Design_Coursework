%% 椭圆轨道到目标圆轨道的 Lambert 参考最优脉冲搜索
%
% 本脚本在二维共面条件下，使用与主方法一致的两脉冲 Lambert 转移，
% 计算从初始椭圆轨道进入目标圆轨道的参考最优速度脉冲。
%
% 与主远程导引脚本的差别在于：
%   1. 此处不再绑定目标相位，也不要求到达固定捕获点。
%   2. 搜索变量改为出发真近点角 theta_d 与转移弧段角度 Delta theta_tr。
%   3. 对每组几何参数，再在一组 Delta t 网格上搜索最优 Lambert 两脉冲。
%
% 输出：
%   1. 最优参考方案的出发真近点角、转移弧段角度与飞行时间
%   2. 最优方案的两次脉冲与总 Delta-v
%   3. 以 theta_d-Delta theta_tr 为横纵轴的最优总 Delta-v 等高线图

close all;
clear;
clc;
utils.setDefaultGraphics();

%% 常数
mu = 3.986004418e14; % m^3/s^2
Re = 6378137; % m
safe_radius = Re + 150e3; % m
deg = pi / 180;

%% 任务场景参数
target = struct();
target.radius = Re + 1000e3; % 目标圆轨道半径
target.speed = sqrt(mu / target.radius);
target.n = sqrt(mu / target.radius^3);
target.T = 2 * pi / target.n;

chaser = struct();
chaser.rp = Re + 400e3; % 初始椭圆轨道近地点半径
chaser.ra = Re + 600e3; % 初始椭圆轨道远地点半径
chaser.a = (chaser.rp + chaser.ra) / 2;
chaser.e = (chaser.ra - chaser.rp) / (chaser.ra + chaser.rp);
chaser.p = chaser.a * (1 - chaser.e^2);

%% 粗搜索网格
search_config = struct();
search_config.theta_departure_list = deg * (0:2:356);
search_config.transfer_arc_list = deg * (8:2:352);
search_config.dt_list = (10 * 60):60:(1.5 * target.T);

%% 局部细化网格
refine_config = struct();
refine_config.theta_half_width = 6 * deg;
refine_config.theta_step = 0.5 * deg;
refine_config.transfer_arc_half_width = 6 * deg;
refine_config.transfer_arc_step = 0.5 * deg;
refine_config.dt_half_width = 5 * 60;
refine_config.dt_step = 10;

%% 全局粗搜索
coarse_search = search_reference_grid( ...
    search_config.theta_departure_list, ...
    search_config.transfer_arc_list, ...
    search_config.dt_list, ...
    mu, safe_radius, chaser, target);

%% 最优点局部细化
[refined_best, refined_search] = refine_best_solution( ...
    coarse_search.best, search_config.dt_list, refine_config, ...
    mu, safe_radius, chaser, target);

best = refined_best;

%% 数据输出
fprintf('初始椭圆轨道参数：\n');
fprintf('  rp = %.3f km\n', chaser.rp / 1e3);
fprintf('  ra = %.3f km\n', chaser.ra / 1e3);
fprintf('  a  = %.3f km\n', chaser.a / 1e3);
fprintf('  e  = %.6f\n', chaser.e);
fprintf('目标圆轨道半径：%.3f km\n', target.radius / 1e3);
fprintf('转移弧段最低高度约束：%.3f km\n\n', (safe_radius - Re) / 1e3);

fprintf('粗搜索可行网格数 = %d / %d (%.2f%%)\n', ...
    coarse_search.feasible_case_count, ...
    coarse_search.total_case_count, ...
    100 * coarse_search.feasible_case_count / coarse_search.total_case_count);
fprintf('最优 Lambert 参考方案如下：\n');
fprintf('  出发真近点角 theta_d = %.2f deg\n', best.theta_departure / deg);
fprintf('  到达圆轨道角位置 theta_a = %.2f deg\n', best.theta_arrival / deg);
fprintf('  转移弧段角度 Delta theta_tr = %.2f deg\n', best.transfer_arc / deg);
fprintf('  飞行时间 Delta t = %.2f s (%.2f min)\n', best.dt, best.dt / 60);
fprintf('  转移支路 = %s\n', best.branch);
fprintf('  起点脉冲 Delta v1 = %.4f m/s\n', best.dv1);
fprintf('  末端圆化脉冲 Delta v2 = %.4f m/s\n', best.dv2);
fprintf('  总 Delta v = %.4f m/s\n', best.dv_total);
fprintf('  转移弧段最低高度 = %.2f km\n', (best.rmin_tr - Re) / 1e3);

if ~isempty(refined_search)
    fprintf('\n局部细化搜索范围：\n');
    fprintf('  theta_d 半宽 = %.2f deg, 步长 = %.2f deg\n', ...
        refine_config.theta_half_width / deg, refine_config.theta_step / deg);
    fprintf('  Delta theta_tr 半宽 = %.2f deg, 步长 = %.2f deg\n', ...
        refine_config.transfer_arc_half_width / deg, refine_config.transfer_arc_step / deg);
    fprintf('  Delta t 半宽 = %.2f min, 步长 = %.2f s\n', ...
        refine_config.dt_half_width / 60, refine_config.dt_step);
end

%% 等高线图
contour_plot_config = struct();
contour_plot_config.figure_name = '椭圆轨道到目标圆轨道 Lambert 参考等高线图';
contour_plot_config.figure_width = 8;
contour_plot_config.aspect_ratio = 0.68;
contour_plot_config.title_text = '椭圆轨道到目标圆轨道 Lambert 参考最优总 \Delta{\itv} 等高线图';
contour_plot_config.xlabel_text = '出发真近点角 theta_d (deg)';
contour_plot_config.ylabel_text = '转移弧段角度 Delta theta_tr (deg)';
contour_plot_config.colorbar_label = '最优总 \Delta{\itv} (m/s)';
contour_plot_config.x_scale = 180 / pi;
contour_plot_config.y_scale = 180 / pi;
contour_plot_config.display_upper_limit = 1000;
contour_plot_config.level_count = 10;
contour_plot_config.contour_width = 1;
contour_plot_config.colorbar_tick_count_max = 6;

contour_plot_info = utils.plotContourMap2D( ...
    search_config.theta_departure_list, ...
    search_config.transfer_arc_list, ...
    coarse_search.dv_total_map, ...
    contour_plot_config);

if contour_plot_info.has_contour_plot
    hold(contour_plot_info.ax, 'on');
    plot(contour_plot_info.ax, ...
        best.theta_departure * contour_plot_config.x_scale, ...
        best.transfer_arc * contour_plot_config.y_scale, ...
        'p', ...
        'MarkerSize', 9, ...
        'MarkerFaceColor', [0.85, 0.15, 0.10], ...
        'MarkerEdgeColor', 'k', ...
        'LineWidth', 1.0);
end

reference_result = struct();
reference_result.mu = mu;
reference_result.Re = Re;
reference_result.safe_radius = safe_radius;
reference_result.chaser = chaser;
reference_result.target = target;
reference_result.search_config = search_config;
reference_result.refine_config = refine_config;
reference_result.coarse_search = coarse_search;
reference_result.refined_search = refined_search;
reference_result.best = best;
reference_result.contour_plot = contour_plot_info;

%% 局部函数
function search = search_reference_grid( ...
    theta_departure_list, transfer_arc_list, dt_list, ...
    mu, safe_radius, chaser, target)
% search_reference_grid 对几何参数与转移时间网格做 Lambert 搜索。

num_theta = numel(theta_departure_list);
num_arc = numel(transfer_arc_list);
num_dt = numel(dt_list);

dv_total_map = nan(num_arc, num_theta);
dv1_map = nan(num_arc, num_theta);
dv2_map = nan(num_arc, num_theta);
dt_opt_map = nan(num_arc, num_theta);
min_radius_map = nan(num_arc, num_theta);
branch_map = strings(num_arc, num_theta);

best = initialize_best_solution();
feasible_case_count = 0;

for i_theta = 1:num_theta
    theta_departure = theta_departure_list(i_theta);
    [r1, v1_nom] = build_elliptic_state_from_true_anomaly( ...
        theta_departure, mu, chaser);

    for i_arc = 1:num_arc
        transfer_arc = transfer_arc_list(i_arc);
        theta_arrival = wrap_to_2pi(theta_departure + transfer_arc);
        branch = select_branch_from_transfer_arc(transfer_arc);
        [r2, vf] = build_circular_state_from_angle( ...
            theta_arrival, mu, target.radius);

        cell_best = initialize_best_solution();
        cell_best.theta_departure = theta_departure;
        cell_best.theta_arrival = theta_arrival;
        cell_best.transfer_arc = transfer_arc;
        cell_best.branch = branch;
        cell_best.r1 = r1;
        cell_best.v1_nom = v1_nom;
        cell_best.r2 = r2;
        cell_best.vf = vf;

        for i_dt = 1:num_dt
            dt = dt_list(i_dt);
            [v1_tr, v2_tr, lambert_info] = lambert_universal_2d( ...
                r1, r2, dt, mu, branch);

            if ~lambert_info.converged
                continue;
            end

            rmin_tr = transfer_arc_min_radius_2d(r1, v1_tr, r2, v2_tr, mu);
            if ~isfinite(rmin_tr) || rmin_tr < safe_radius
                continue;
            end

            dv1_vec = v1_tr - v1_nom;
            dv2_vec = vf - v2_tr;
            dv1 = norm(dv1_vec);
            dv2 = norm(dv2_vec);
            dv_total = dv1 + dv2;

            if dv_total < cell_best.dv_total
                cell_best.dv_total = dv_total;
                cell_best.dv1 = dv1;
                cell_best.dv2 = dv2;
                cell_best.dv1_vec = dv1_vec;
                cell_best.dv2_vec = dv2_vec;
                cell_best.dt = dt;
                cell_best.v1_tr = v1_tr;
                cell_best.v2_tr = v2_tr;
                cell_best.rmin_tr = rmin_tr;
                cell_best.lambert_info = lambert_info;
            end
        end

        if isfinite(cell_best.dv_total)
            feasible_case_count = feasible_case_count + 1;
            dv_total_map(i_arc, i_theta) = cell_best.dv_total;
            dv1_map(i_arc, i_theta) = cell_best.dv1;
            dv2_map(i_arc, i_theta) = cell_best.dv2;
            dt_opt_map(i_arc, i_theta) = cell_best.dt;
            min_radius_map(i_arc, i_theta) = cell_best.rmin_tr;
            branch_map(i_arc, i_theta) = cell_best.branch;

            if cell_best.dv_total < best.dv_total
                best = cell_best;
            end
        end
    end
end

if ~isfinite(best.dv_total)
    error('给定搜索网格内未找到满足最低高度约束的 Lambert 参考解。');
end

search = struct();
search.theta_departure_list = theta_departure_list;
search.transfer_arc_list = transfer_arc_list;
search.dt_list = dt_list;
search.dv_total_map = dv_total_map;
search.dv1_map = dv1_map;
search.dv2_map = dv2_map;
search.dt_opt_map = dt_opt_map;
search.min_radius_map = min_radius_map;
search.branch_map = branch_map;
search.total_case_count = num_theta * num_arc;
search.feasible_case_count = feasible_case_count;
search.best = best;

end

function [best_refined, refined_search] = refine_best_solution( ...
    coarse_best, coarse_dt_list, refine_config, ...
    mu, safe_radius, chaser, target)
% refine_best_solution 在粗搜索最优点附近做局部细化。

theta_departure_list = build_wrapped_angle_window( ...
    coarse_best.theta_departure, ...
    refine_config.theta_half_width, ...
    refine_config.theta_step);

transfer_arc_min = max(1 * pi / 180, ...
    coarse_best.transfer_arc - refine_config.transfer_arc_half_width);
transfer_arc_max = min(2 * pi - 1 * pi / 180, ...
    coarse_best.transfer_arc + refine_config.transfer_arc_half_width);
transfer_arc_list = transfer_arc_min:refine_config.transfer_arc_step:transfer_arc_max;

if isempty(transfer_arc_list)
    transfer_arc_list = coarse_best.transfer_arc;
end

dt_min = max(coarse_dt_list(1), coarse_best.dt - refine_config.dt_half_width);
dt_max = min(coarse_dt_list(end), coarse_best.dt + refine_config.dt_half_width);
dt_list = dt_min:refine_config.dt_step:dt_max;

if isempty(dt_list)
    dt_list = coarse_best.dt;
end

refined_search = [];
best_refined = coarse_best;

refined_search = search_reference_grid( ...
    theta_departure_list, transfer_arc_list, dt_list, ...
    mu, safe_radius, chaser, target);
best_refined = refined_search.best;


end

function [r_vec, v_vec] = build_elliptic_state_from_true_anomaly(theta, mu, chaser)
% build_elliptic_state_from_true_anomaly 由真近点角计算椭圆轨道状态。

radius = chaser.p / (1 + chaser.e * cos(theta));
r_vec = radius * [cos(theta); sin(theta)];
v_vec = sqrt(mu / chaser.p) * [-sin(theta); chaser.e + cos(theta)];

end

function [r_vec, v_vec] = build_circular_state_from_angle(theta, mu, radius)
% build_circular_state_from_angle 由极角计算圆轨道状态。

speed = sqrt(mu / radius);
r_vec = radius * [cos(theta); sin(theta)];
v_vec = speed * [-sin(theta); cos(theta)];

end

function branch = select_branch_from_transfer_arc(transfer_arc)
% select_branch_from_transfer_arc 按给定转移弧段角度选择 Lambert 支路。

if transfer_arc <= pi
    branch = "short";
else
    branch = "long";
end

end

function best = initialize_best_solution()
% initialize_best_solution 构造默认最优解结构体。

best = struct( ...
    'dv_total', inf, ...
    'dv1', nan, ...
    'dv2', nan, ...
    'dv1_vec', nan(2, 1), ...
    'dv2_vec', nan(2, 1), ...
    'theta_departure', nan, ...
    'theta_arrival', nan, ...
    'transfer_arc', nan, ...
    'dt', nan, ...
    'branch', "", ...
    'r1', nan(2, 1), ...
    'v1_nom', nan(2, 1), ...
    'r2', nan(2, 1), ...
    'vf', nan(2, 1), ...
    'v1_tr', nan(2, 1), ...
    'v2_tr', nan(2, 1), ...
    'rmin_tr', nan, ...
    'lambert_info', struct());

end

function theta_list = build_wrapped_angle_window(center, half_width, step)
% build_wrapped_angle_window 构造围绕中心角的局部环形搜索窗口。

offset_num = round(half_width / step);
offset_list = (-offset_num:offset_num) * step;
theta_list = wrap_to_2pi(center + offset_list);
theta_list = unique(theta_list, 'stable');

end

function angle_wrapped = wrap_to_2pi(angle_value)
% wrap_to_2pi 将角度映射到 [0, 2*pi)。

angle_wrapped = mod(angle_value, 2 * pi);

end
