%% 插入平面自然绕飞轨道的二维搜索
%
% 本脚本使用 CW 状态转移矩阵，在固定平面自然绕飞轨道上搜索
% 最优两脉冲插入方案。
%
% 搜索变量：
%   1. 转移时间 T
%   2. 终端绕飞相位 theta_f
%
% 平面自然绕飞轨道定义为：
%   x = k * sin(theta_f)
%   y = 2 * k * cos(theta_f) + yc

clear;
clc;
utils.setDefaultGraphics();

%% 常数
mu = 3.986004418e14; % 地球引力参数
Re = 6378137;
h = 1000e3;
r = Re + h;

n = sqrt(mu / r^3);
Tc = 2 * pi / n;

%% 绕飞轨道参数
k = 1.5e3;
yc = 0;

%% 转移初始状态
R0 = [-2000; -2000; 0];
V0 = [10; 10; 0];

%% 二维搜索参数
T_min = 100;
T_max = 2 * Tc;
T_steps = 500;
T_list = linspace(T_min, T_max, T_steps);

theta_steps = 720;
theta_list = linspace(0, 2 * pi, theta_steps + 1);
theta_list(end) = [];

%% 可视化参数
dv_vis_cap = 30; % 仅用于绘图显示的 Δv 截断上限

%% 二维搜索
stats = analyze_transfer_2d(n, R0, V0, k, yc, T_list, theta_list);

%% 输出结果
disp("==== 绕飞轨道参数 ====");
fprintf('轨道周期 Tc = %.2f s\n', Tc);
fprintf('k = %.3f m, yc = %.3f m\n', k, yc);
fprintf('搜索时间范围 = [%.2f, %.2f] s\n', T_min, T_max);
fprintf('时间离散数 = %d\n', T_steps);
fprintf('相位离散数 = %d\n\n', theta_steps);

disp("==== 最优两脉冲插入结果 ====");
fprintf('最优插入时间 T = %.2f s\n', stats.T_opt);
fprintf('最优终端相位 theta_f = %.6f rad (%.2f deg)\n', ...
    stats.theta_opt, stats.theta_opt * 180 / pi);
fprintf('插入点位置 = [%.3f %.3f %.3f] m\n', stats.rf_opt);
fprintf('插入点速度 = [%.6f %.6f %.6f] m/s\n', stats.vf_opt);
fprintf('Δv1 = [%.6f %.6f %.6f] m/s\n', stats.dv1_opt);
fprintf('Δv2 = [%.6f %.6f %.6f] m/s\n', stats.dv2_opt);
fprintf('总 Δv = %.6f m/s\n', stats.dv_total_opt);

%% 最优转移轨迹
traj_transfer = generate_transfer_trajectory( ...
    n, R0, stats.v0_plus_opt, stats.T_opt, 300);

theta_plot = linspace(0, 2 * pi, 500);
traj_circ = zeros(3, numel(theta_plot));

for i = 1:numel(theta_plot)
    [r_circ, ~] = circ_state(n, k, yc, theta_plot(i));
    traj_circ(:, i) = r_circ;
end

%% 绘图：最优插入轨迹
utils.createFigureA4();
hold on;
grid on;
axis equal;

plot(traj_circ(1, :), traj_circ(2, :), 'r--', 'LineWidth', 1.5);
plot(traj_transfer(1, :), traj_transfer(2, :), 'b-', 'LineWidth', 1.5);

scatter(0, 0, 60, 'k', 'filled');
scatter(R0(1), R0(2), 60, 'g', 'filled');
scatter(stats.rf_opt(1), stats.rf_opt(2), 60, 'r', 'filled');

xlabel('x (m)');
ylabel('y (m)');
legend(utils.formatMixedFontText({'自然绕飞轨道', '最优两脉冲转移轨迹', ...
    '目标飞行器', '转移初始点', '最优插入点'}));
title(utils.formatMixedFontText('CW 两脉冲插入平面自然绕飞轨道'));

%% 绘图：二维搜索结果
theta_deg = theta_list * 180 / pi;
[T_grid, theta_grid] = meshgrid(T_list / 60, theta_deg);
dv_total_map_vis = stats.dv_total_map;
dv_total_map_vis(dv_total_map_vis > dv_vis_cap) = dv_vis_cap;

dv_vis_valid = dv_total_map_vis(isfinite(dv_total_map_vis));
dv_vis_min = min(dv_vis_valid);

utils.createFigureA4();
contourf(T_grid, theta_grid, dv_total_map_vis, 30, 'LineColor', 'none');
hold on;
grid on;
if ~isempty(which('utils.applyViridisColormap'))
    utils.applyViridisColormap(256, true);
else
    colormap(flipud(parula(256)));
end
clim([dv_vis_min, dv_vis_cap]);

cb = colorbar;
cb.Label.String = utils.formatMixedFontText( ...
    sprintf('显示用截断后 \\Delta{\\itv} (m/s), cap = %.1f', dv_vis_cap));

scatter(stats.T_opt / 60, stats.theta_opt * 180 / pi, ...
    45, 'w', 'filled', 'MarkerEdgeColor', 'k');

xlabel('Transfer Time (min)');
ylabel('\theta_f (deg)');
title(utils.formatMixedFontText('二维搜索下的总 \Delta{\itv} 分布（高值已截断）'));

%% 绘图：每个时间对应的最优终端相位
utils.createFigureA4();
hold on;
grid on;

plot(T_list / 60, stats.dv_min_over_theta, 'k', 'LineWidth', 1.5);
scatter(stats.T_opt / 60, stats.dv_total_opt, 45, 'k', 'filled');

xlabel('Transfer Time (min)');
ylabel('Min \Delta{\itv} over \theta_f (m/s)');
title(utils.formatMixedFontText('对终端相位取最优后的 \Delta{\itv}-T 曲线'));

%% 局部函数
function stats = analyze_transfer_2d(n, R0, V0, k, yc, T_list, theta_list)

arguments
    n(1, 1) double{mustBePositive}
    R0(3, 1) double
    V0(3, 1) double
    k(1, 1) double{mustBePositive}
    yc(1, 1) double
    T_list(1, :) double{mustBePositive}
    theta_list(1, :) double
end

dv_total_map = nan(numel(theta_list), numel(T_list));

best_total = inf;
best_i = NaN;
best_j = NaN;

for i = 1:numel(T_list)
    T = T_list(i);
    [Phi_rr, Phi_rv, Phi_vr, Phi_vv] = cw_stm(n, T);

    if rcond(Phi_rv) < 1e-10
        continue;
    end

    for j = 1:numel(theta_list)
        theta_f = theta_list(j);
        [rf, vf] = circ_state(n, k, yc, theta_f);

        v0_plus = Phi_rv \ (rf - Phi_rr * R0);
        vf_minus = Phi_vr * R0 + Phi_vv * v0_plus;

        dv1 = v0_plus - V0;
        dv2 = vf - vf_minus;
        dv_total = norm(dv1) + norm(dv2);

        dv_total_map(j, i) = dv_total;

        if dv_total < best_total
            best_total = dv_total;
            best_i = i;
            best_j = j;

            best = struct();
            best.T = T;
            best.theta = theta_f;
            best.rf = rf;
            best.vf = vf;
            best.dv1 = dv1;
            best.dv2 = dv2;
            best.v0_plus = v0_plus;
        end
    end
end

if isnan(best_i)
    error('给定二维搜索区间内未找到可逆的两脉冲插入解。');
end

dv_min_over_theta = nan(1, numel(T_list));

for i = 1:numel(T_list)
    col = dv_total_map(:, i);
    col = col(~isnan(col));

    if ~isempty(col)
        dv_min_over_theta(i) = min(col);
    end
end

stats = struct();
stats.T_opt = best.T;
stats.theta_opt = best.theta;
stats.rf_opt = best.rf;
stats.vf_opt = best.vf;
stats.dv1_opt = best.dv1;
stats.dv2_opt = best.dv2;
stats.v0_plus_opt = best.v0_plus;
stats.dv_total_opt = best_total;
stats.dv_total_map = dv_total_map;
stats.dv_min_over_theta = dv_min_over_theta;
stats.idx_T_opt = best_i;
stats.idx_theta_opt = best_j;
end

function [r_t, v_t] = circ_state(n, k, yc, theta)

r_t = [k * sin(theta); 2 * k * cos(theta) + yc; 0];
v_t = [k * n * cos(theta); -2 * k * n * sin(theta); 0];

end

function traj = generate_transfer_trajectory(n, R0, v0_plus, T, steps)

tspan = linspace(0, T, steps);
traj = zeros(3, numel(tspan));

for i = 1:numel(tspan)
    [Phi_rr, Phi_rv, ~, ~] = cw_stm(n, tspan(i));
    traj(:, i) = Phi_rr * R0 + Phi_rv * v0_plus;
end

end
