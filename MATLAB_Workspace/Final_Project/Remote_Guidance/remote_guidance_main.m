%% 远程导引段 Lambert 窗口搜索与可视化
%
% 本脚本在二维共面条件下，直接在跟踪飞行器初始椭圆轨道上搜索
% 远程导引段的 Lambert 转移窗口。搜索变量为出发时刻 t_d 与飞行时间 Δt。
% 当前 Lambert 求解器仅覆盖零圈（0-rev）short-way / long-way 两支。
% 终端点固定为目标飞行器 LVLH 坐标系中的 [0; -5 km] 捕获点。
%
% 输出：
%   1. 满足转移弧段高度安全约束的 t_d-Δt 等高线图，颜色表示总 Δv
%   2. 当前搜索网格中的最优 Lambert 转移窗口
%   3. 供后续近距离导引使用的捕获点状态信息
%   4. 最优窗口对应的等待段与转移段轨迹图
%   5. 最优窗口对应的远程导引转移动画与 GIF 文件

close all;
clear;
clc;
utils.setDefaultGraphics();

%% 输出目录
script_dir = fileparts(mfilename('fullpath'));
output_dir = fullfile(script_dir, '..', '..', 'output');

if ~isfolder(output_dir)
    mkdir(output_dir);
end

%% 常数
mu = 3.986004418e14; % m^3/s^2
Re = 6378137; % m
safe_radius = Re + 150e3; % m
deg = pi / 180;

%% 任务场景参数
target = struct();
target.radius = Re + 1000e3; % 目标圆轨道半径
target.theta0 = 80 * deg; % 目标初始相位角
target.t0 = 0;
target.n = sqrt(mu / target.radius^3);
target.T = 2 * pi / target.n;

chaser = struct();
chaser.rp = Re + 400e3; % 跟踪器近地点半径
chaser.ra = Re + 600e3; % 跟踪器远地点半径
chaser.a = (chaser.rp + chaser.ra) / 2;
chaser.e = (chaser.ra - chaser.rp) / (chaser.ra + chaser.rp);
chaser.p = chaser.a * (1 - chaser.e^2);
chaser.theta0 = 45 * deg; % 初始真近点角
chaser.t0 = 0;
chaser.E0 = 2 * atan2( ...
    sqrt(1 - chaser.e) * sin(chaser.theta0 / 2), ...
    sqrt(1 + chaser.e) * cos(chaser.theta0 / 2));
chaser.M0 = chaser.E0 - chaser.e * sin(chaser.E0);
chaser.n = sqrt(mu / chaser.a^3);
chaser.T = 2 * pi / chaser.n;

rho_cap_L = [0; -5e3]; % LVLH 捕获点
rhodot_cap_L = [0; 0]; % 先对准为静止捕获点

%% 搜索网格
td_min = 0;
td_max = 1.5 * target.T;
dt_min = 10 * 60;
dt_max = 1.2 * target.T;
td_step = 60;
dt_step = 60;

td_list = td_min:td_step:td_max;
dt_list = dt_min:dt_step:dt_max;
branch_names = ["short", "long"]; % 当前仅搜索零圈 Lambert 的几何两支

%% 预分配
num_td = numel(td_list);
num_dt = numel(dt_list);

dv_total_map = nan(num_dt, num_td);
dv1_map = nan(num_dt, num_td);
dv2_map = nan(num_dt, num_td);
arrival_rel_speed_map = nan(num_dt, num_td);
min_radius_map = nan(num_dt, num_td);
branch_map = strings(num_dt, num_td);

best = struct( ...
    'dv_total', inf, ...
    'dv1', nan, ...
    'dv2', nan, ...
    'dv1_vec', nan(2, 1), ...
    'dv2_vec', nan(2, 1), ...
    'td', nan, ...
    'dt', nan, ...
    'ta', nan, ...
    'branch', "", ...
    'r1', nan(2, 1), ...
    'v1_nom', nan(2, 1), ...
    'r2', nan(2, 1), ...
    'vf', nan(2, 1), ...
    'v1_tr', nan(2, 1), ...
    'v2_tr', nan(2, 1), ...
    'rmin_tr', nan, ...
    'arrival_rel_speed', nan, ...
    'rho_cap_L', rho_cap_L, ...
    'rhodot_cap_L', rhodot_cap_L);

S = [0, -1; 1, 0];

%% Lambert 窗口搜索
for i_td = 1:num_td
    td = td_list(i_td);
    [r1, v1_nom] = chaser_state_elliptic_2d(td, mu, chaser);

    for i_dt = 1:num_dt
        dt = dt_list(i_dt);
        ta = td + dt;

        [r_ta, v_ta] = target_state_circular_2d(ta, mu, target);
        [r2, vf, capture_info] = capture_state_from_target_2d( ...
            r_ta, v_ta, rho_cap_L, rhodot_cap_L);

        cell_best_dv = inf;
        cell_best = struct();

        for i_branch = 1:numel(branch_names)
            branch = branch_names(i_branch);
            [v1_tr, v2_tr, lambert_info] = lambert_universal_2d(r1, r2, dt, mu, branch);

            if ~lambert_info.converged
                continue;
            end

            rmin_tr = transfer_arc_min_radius_2d( ...
                r1, v1_tr, r2, v2_tr, mu);
            if ~isfinite(rmin_tr) || rmin_tr < safe_radius
                continue;
            end

            dv1_vec = v1_tr - v1_nom;
            dv2_vec = vf - v2_tr;
            dv1 = norm(dv1_vec);
            dv2 = norm(dv2_vec);
            dv_total = dv1 + dv2;

            rho_dot_minus = capture_info.C_I2L * (v2_tr - v_ta) ...
                - capture_info.n * S * rho_cap_L;
            arrival_rel_speed = norm(rho_dot_minus);

            if dv_total < cell_best_dv
                cell_best_dv = dv_total;
                cell_best.dv_total = dv_total;
                cell_best.dv1 = dv1;
                cell_best.dv2 = dv2;
                cell_best.dv1_vec = dv1_vec;
                cell_best.dv2_vec = dv2_vec;
                cell_best.td = td;
                cell_best.dt = dt;
                cell_best.ta = ta;
                cell_best.branch = branch;
                cell_best.r1 = r1;
                cell_best.v1_nom = v1_nom;
                cell_best.r2 = r2;
                cell_best.vf = vf;
                cell_best.v1_tr = v1_tr;
                cell_best.v2_tr = v2_tr;
                cell_best.rmin_tr = rmin_tr;
                cell_best.arrival_rel_speed = arrival_rel_speed;
                cell_best.rho_cap_L = rho_cap_L;
                cell_best.rhodot_cap_L = rhodot_cap_L;
            end
        end

        if isfinite(cell_best_dv)
            dv_total_map(i_dt, i_td) = cell_best.dv_total;
            dv1_map(i_dt, i_td) = cell_best.dv1;
            dv2_map(i_dt, i_td) = cell_best.dv2;
            arrival_rel_speed_map(i_dt, i_td) = cell_best.arrival_rel_speed;
            min_radius_map(i_dt, i_td) = cell_best.rmin_tr;
            branch_map(i_dt, i_td) = cell_best.branch;

            if cell_best.dv_total < best.dv_total
                best = cell_best;
            end
        end
    end
end

%% 数值递推验证
simulation_result = [];
simulation_config = struct();

if isfinite(best.dv_total)
    simulation_config.wait_step = 30; % s
    simulation_config.transfer_step = 20; % s
    simulation_config.reference_orbit_samples = 720;

    simulation_result = simulate_impulsive_remote_guidance_2d( ...
        mu, Re, target, chaser, best, rho_cap_L, rhodot_cap_L, simulation_config);
else
    warning('当前搜索网格内未找到满足转移弧段高度约束的 Lambert 可行解。');
end

%% 数据输出
num_total_cells = numel(dv_total_map);
num_safe_cells = nnz(isfinite(dv_total_map));
safe_ratio = num_safe_cells / num_total_cells;

fprintf('满足转移弧段最低高度约束的网格数 = %d / %d (%.2f%%)\n', ...
    num_safe_cells, num_total_cells, 100 * safe_ratio);

if isfinite(best.dv_total)
    fprintf('最优 Lambert 窗口如下：\n');
    fprintf('t_d = %.2f s (%.2f min)\n', best.td, best.td / 60);
    fprintf('Δt = %.2f s (%.2f min)\n', best.dt, best.dt / 60);
    fprintf('t_a = %.2f s (%.2f min)\n', best.ta, best.ta / 60);
    fprintf('branch = %s\n', best.branch);
    fprintf('Δv1 = %.4f m/s\n', best.dv1);
    fprintf('Δv2 = %.4f m/s\n', best.dv2);
    fprintf('Δv_total = %.4f m/s\n', best.dv_total);
    fprintf('到达脉冲前相对速度 = %.4f m/s\n', best.arrival_rel_speed);
    fprintf('转移弧段最低高度 = %.2f km\n', (best.rmin_tr - Re) / 1e3);

    fprintf('\n========================================\n');
    fprintf('数值递推验证：\n');
    fprintf('等待段末位置误差 = %.6f m\n', simulation_result.validation.departure_position_error);
    fprintf('等待段末速度误差 = %.6f m/s\n', simulation_result.validation.departure_velocity_error);
    fprintf('到达捕获点位置误差 = %.6f m\n', simulation_result.validation.arrival_position_error);
    fprintf('到达转移末速度误差 = %.6f m/s\n', simulation_result.validation.arrival_velocity_error);
    fprintf('数值递推得到的末端脉冲 = %.6f m/s\n', simulation_result.validation.dv2_numeric);
end

%% 图 1：远程导引 Lambert 窗口等高线图
contour_plot_config = struct();
contour_plot_config.figure_name = '远程导引 Lambert 窗口等高线图';
contour_plot_config.figure_width = 20;
contour_plot_config.aspect_ratio = 0.68;
contour_plot_config.title_text = '二维共面远程导引 Lambert 窗口等高线图';
contour_plot_config.xlabel_text = '出发时刻 {\itt}_{\itd} (h)';
contour_plot_config.ylabel_text = '飞行时间 \Delta{\itt} (min)';
contour_plot_config.colorbar_label = '总 \Delta{\itv} (m/s)';
contour_plot_config.x_scale = 1 / 3600;
contour_plot_config.y_scale = 1 / 60;
contour_plot_config.display_upper_limit = 1000;
contour_plot_config.level_count = 10;
contour_plot_config.contour_width = 1;
contour_plot_config.colorbar_tick_count_max = 6;

contour_plot_info = utils.plotContourMap2D( ...
    td_list, dt_list, dv_total_map, contour_plot_config);

if contour_plot_info.has_contour_plot
    fprintf('等高线显示 Δv 上限 = %.4f m/s（按绝对阈值筛除更高区域）\n', ...
        contour_plot_info.contour_info.display_max);
end

%% 图 2：最优转移轨迹图
fig_remote_trajectory = [];

if isfinite(best.dv_total)
    fig_remote_trajectory = plot_remote_guidance_trajectory_2d(simulation_result);
end

%% 图 3：远程导引转移动画
animation_info = [];

if isfinite(best.dv_total)
    animation_config = struct();
    animation_config.frame_count = 240;
    animation_config.pause_seconds = 0.02;
    animation_config.gif_delay = 0.04;
    animation_config.save_gif = true;
    animation_config.gif_path = fullfile(output_dir, 'remote_guidance_transfer.gif');

    animation_info = render_remote_guidance_animation_2d( ...
        simulation_result, animation_config);
end

%% 汇总输出结构体
remote_result = struct();
remote_result.mu = mu;
remote_result.Re = Re;
remote_result.safe_radius = safe_radius;
remote_result.target = target;
remote_result.chaser = chaser;
remote_result.rho_cap_L = rho_cap_L;
remote_result.rhodot_cap_L = rhodot_cap_L;
remote_result.td_list = td_list;
remote_result.dt_list = dt_list;
remote_result.dv_total_map = dv_total_map;
remote_result.dv1_map = dv1_map;
remote_result.dv2_map = dv2_map;
remote_result.arrival_rel_speed_map = arrival_rel_speed_map;
remote_result.min_radius_map = min_radius_map;
remote_result.min_altitude_map = min_radius_map - Re;
remote_result.branch_map = branch_map;
remote_result.num_total_cells = num_total_cells;
remote_result.num_safe_cells = num_safe_cells;
remote_result.safe_ratio = safe_ratio;
remote_result.best = best;
remote_result.contour_plot = contour_plot_info;
remote_result.simulation = simulation_result;
remote_result.simulation_config = simulation_config;
remote_result.trajectory_figure = fig_remote_trajectory;
remote_result.animation = animation_info;
