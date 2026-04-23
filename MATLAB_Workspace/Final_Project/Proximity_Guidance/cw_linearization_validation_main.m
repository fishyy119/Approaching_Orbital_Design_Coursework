%% 近距离导引初始相对状态下的 CW 线性化误差验证
%
% 本脚本保持近距离导引主脚本中的目标轨道与初始相对状态不变，
% 将无控跟踪器与目标飞行器分别在惯性系中做二维两体数值积分，
% 再逐点转换回 LVLH 坐标系，与同初值下的 CW 解析解做对比。
%
% 输出：
%   1. LVLH 坐标系中的 CW 解析轨迹与数值积分轨迹对比图
%   2. LVLH 坐标系中的 x / y 位置误差随时间变化图
%   3. LVLH 坐标系中的 vx / vy 速度误差随时间变化图
%   4. 控制台误差统计信息

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
target.T = 20 * pi / target.n;

%% 初始相对状态
initial_relative = struct();
initial_relative.rel_r0 = [5e3; -5e3];
initial_relative.rel_v0 = [0; 0];

%% 无控验证配置
validation_config = struct();
validation_config.duration = target.T;
validation_config.step = 10;

%% CW 解析解与惯性系数值积分
[target_pos_0, target_vel_0] = fpkinematics.targetStateCircular2D(target.t0, mu, target);
[chaser_pos_0, chaser_vel_0] = fpkinematics.lvlhToInertial2D( ...
    target_pos_0, target_vel_0, initial_relative.rel_r0, initial_relative.rel_v0);

time_list = fpkinematics.buildTimeGrid( ...
    validation_config.duration, validation_config.step);
time_list_min = time_list / 60;

[target_pos_numeric, target_vel_numeric] = fpkinematics.propagateTwoBodyOde1132D( ...
    target_pos_0, target_vel_0, time_list, mu);
[chaser_pos_numeric, chaser_vel_numeric] = fpkinematics.propagateTwoBodyOde1132D( ...
    chaser_pos_0, chaser_vel_0, time_list, mu);
[rel_r_numeric, rel_v_numeric] = fpkinematics.relativeHistoryFromInertial2D( ...
    target_pos_numeric, target_vel_numeric, chaser_pos_numeric, chaser_vel_numeric);
[rel_r_cw, rel_v_cw] = generateCwHistory2D( ...
    target.n, initial_relative.rel_r0, initial_relative.rel_v0, time_list);

position_error = rel_r_numeric - rel_r_cw;
velocity_error = rel_v_numeric - rel_v_cw;
position_error_norm = vecnorm(position_error, 2, 1);
velocity_error_norm = vecnorm(velocity_error, 2, 1);

position_stats = summarizeErrorHistory(position_error, position_error_norm, time_list);
velocity_stats = summarizeErrorHistory(velocity_error, velocity_error_norm, time_list);

%% 数据输出
disp("==== CW 线性化无控漂移验证 ====");
fprintf('目标圆轨道半径 = %.3f km\n', target.radius / 1e3);
fprintf('目标轨道周期 = %.2f s (%.2f min)\n', target.T, target.T / 60);
fprintf('验证时长 = %.2f s (%.2f min)\n', ...
    validation_config.duration, validation_config.duration / 60);
fprintf('采样点数 = %d\n', numel(time_list));
fprintf('初始相对位置 = [%.3f %.3f] km\n', initial_relative.rel_r0 / 1e3);
fprintf('初始相对速度 = [%.4f %.4f] m/s\n', initial_relative.rel_v0);

disp(" ");
disp("---- LVLH 位置误差统计 ----");
fprintf('x 向最大绝对误差 = %.6e m\n', position_stats.max_abs(1));
fprintf('y 向最大绝对误差 = %.6e m\n', position_stats.max_abs(2));
fprintf('x 向 RMS 误差 = %.6e m\n', position_stats.rms(1));
fprintf('y 向 RMS 误差 = %.6e m\n', position_stats.rms(2));
fprintf('位置误差范数最大值 = %.6e m @ t = %.2f min\n', ...
    position_stats.max_norm, position_stats.max_norm_time / 60);
fprintf('末端位置误差 = [%.6e %.6e] m\n', position_stats.final);

disp(" ");
disp("---- LVLH 速度误差统计 ----");
fprintf('vx 向最大绝对误差 = %.6e m/s\n', velocity_stats.max_abs(1));
fprintf('vy 向最大绝对误差 = %.6e m/s\n', velocity_stats.max_abs(2));
fprintf('vx 向 RMS 误差 = %.6e m/s\n', velocity_stats.rms(1));
fprintf('vy 向 RMS 误差 = %.6e m/s\n', velocity_stats.rms(2));
fprintf('速度误差范数最大值 = %.6e m/s @ t = %.2f min\n', ...
    velocity_stats.max_norm, velocity_stats.max_norm_time / 60);
fprintf('末端速度误差 = [%.6e %.6e] m/s\n', velocity_stats.final);

%% 图 1：CW 解析轨迹与数值积分轨迹对比图
trajectory_plot_config = struct();
trajectory_plot_config.figure_name = '近距离导引 CW 解析轨迹与数值积分轨迹对比图';
trajectory_plot_config.figure_width = 20;
trajectory_plot_config.aspect_ratio = 0.72;

fig_trajectory = utils.createFigureA4(struct( ...
    'Name', trajectory_plot_config.figure_name, ...
    'Width', trajectory_plot_config.figure_width, ...
    'AspectRatio', trajectory_plot_config.aspect_ratio));
ax_trajectory = axes('Parent', fig_trajectory);
hold(ax_trajectory, 'on');
grid(ax_trajectory, 'on');
box(ax_trajectory, 'on');

plot(ax_trajectory, rel_r_cw(1, :) / 1e3, rel_r_cw(2, :) / 1e3, ...
    '--', ...
    'Color', [0.88, 0.33, 0.12], ...
    'LineWidth', 1.6, ...
    'DisplayName', utils.formatMixedFontText('CW 解析轨迹'));
plot(ax_trajectory, rel_r_numeric(1, :) / 1e3, rel_r_numeric(2, :) / 1e3, ...
    '-', ...
    'Color', [0.10, 0.35, 0.85], ...
    'LineWidth', 1.8, ...
    'DisplayName', utils.formatMixedFontText('数值积分轨迹'));
plot(ax_trajectory, initial_relative.rel_r0(1) / 1e3, initial_relative.rel_r0(2) / 1e3, ...
    'o', ...
    'MarkerSize', 6.5, ...
    'MarkerFaceColor', [0.10, 0.35, 0.85], ...
    'MarkerEdgeColor', 'k', ...
    'DisplayName', utils.formatMixedFontText('初始点'));

xlabel(ax_trajectory, utils.formatMixedFontText('LVLH x (km)'));
ylabel(ax_trajectory, utils.formatMixedFontText('LVLH y (km)'));
utils.applyPlotTitle(ax_trajectory, utils.formatMixedFontText('CW 解析轨迹与数值积分轨迹对比'));
legend(ax_trajectory, 'Location', 'bestoutside');

%% 图 2：LVLH 坐标系位置误差变化图
position_error_plot_config = struct();
position_error_plot_config.figure_name = '近距离导引 LVLH 位置误差变化图';
position_error_plot_config.figure_width = 20;
position_error_plot_config.aspect_ratio = 0.68;

fig_position_error = utils.createFigureA4(struct( ...
    'Name', position_error_plot_config.figure_name, ...
    'Width', position_error_plot_config.figure_width, ...
    'AspectRatio', position_error_plot_config.aspect_ratio));
ax_position_error = axes('Parent', fig_position_error);
hold(ax_position_error, 'on');
grid(ax_position_error, 'on');
box(ax_position_error, 'on');

yline(ax_position_error, 0, ':', ...
    'Color', [0.55, 0.55, 0.55], ...
    'HandleVisibility', 'off');
plot(ax_position_error, time_list_min, position_error(1, :), ...
    '-', ...
    'Color', [0.10, 0.35, 0.85], ...
    'LineWidth', 1.8, ...
    'DisplayName', utils.formatMixedFontText('x 方向误差'));
plot(ax_position_error, time_list_min, position_error(2, :), ...
    '-', ...
    'Color', [0.90, 0.45, 0.10], ...
    'LineWidth', 1.8, ...
    'DisplayName', utils.formatMixedFontText('y 方向误差'));

xlim(ax_position_error, [time_list_min(1), time_list_min(end)]);
xlabel(ax_position_error, utils.formatMixedFontText('时间 (min)'));
ylabel(ax_position_error, utils.formatMixedFontText('位置误差 (m)'));
utils.applyPlotTitle(ax_position_error, utils.formatMixedFontText('LVLH 坐标系位置误差分量变化'));
legend(ax_position_error, 'Location', 'best');

%% 图 3：LVLH 坐标系速度误差变化图
velocity_error_plot_config = struct();
velocity_error_plot_config.figure_name = '近距离导引 LVLH 速度误差变化图';
velocity_error_plot_config.figure_width = 20;
velocity_error_plot_config.aspect_ratio = 0.68;

fig_velocity_error = utils.createFigureA4(struct( ...
    'Name', velocity_error_plot_config.figure_name, ...
    'Width', velocity_error_plot_config.figure_width, ...
    'AspectRatio', velocity_error_plot_config.aspect_ratio));
ax_velocity_error = axes('Parent', fig_velocity_error);
hold(ax_velocity_error, 'on');
grid(ax_velocity_error, 'on');
box(ax_velocity_error, 'on');

yline(ax_velocity_error, 0, ':', ...
    'Color', [0.55, 0.55, 0.55], ...
    'HandleVisibility', 'off');
plot(ax_velocity_error, time_list_min, velocity_error(1, :), ...
    '-', ...
    'Color', [0.10, 0.35, 0.85], ...
    'LineWidth', 1.8, ...
    'DisplayName', utils.formatMixedFontText('vx 方向误差'));
plot(ax_velocity_error, time_list_min, velocity_error(2, :), ...
    '-', ...
    'Color', [0.90, 0.45, 0.10], ...
    'LineWidth', 1.8, ...
    'DisplayName', utils.formatMixedFontText('vy 方向误差'));

xlim(ax_velocity_error, [time_list_min(1), time_list_min(end)]);
xlabel(ax_velocity_error, utils.formatMixedFontText('时间 (min)'));
ylabel(ax_velocity_error, utils.formatMixedFontText('速度误差 (m/s)'));
utils.applyPlotTitle(ax_velocity_error, utils.formatMixedFontText('LVLH 坐标系速度误差分量变化'));
legend(ax_velocity_error, 'Location', 'best');

%% 汇总输出结构体
cw_linearization_validation_result = struct();
cw_linearization_validation_result.mu = mu;
cw_linearization_validation_result.Re = Re;
cw_linearization_validation_result.target = target;
cw_linearization_validation_result.initial_relative = initial_relative;
cw_linearization_validation_result.validation_config = validation_config;
cw_linearization_validation_result.time = struct( ...
    'seconds', time_list, ...
    'minutes', time_list_min);
cw_linearization_validation_result.inertial = struct( ...
    'target_pos', target_pos_numeric, ...
    'target_vel', target_vel_numeric, ...
    'chaser_pos', chaser_pos_numeric, ...
    'chaser_vel', chaser_vel_numeric);
cw_linearization_validation_result.relative = struct( ...
    'numeric_r', rel_r_numeric, ...
    'numeric_v', rel_v_numeric, ...
    'cw_r', rel_r_cw, ...
    'cw_v', rel_v_cw);
cw_linearization_validation_result.error = struct( ...
    'position', position_error, ...
    'velocity', velocity_error, ...
    'position_norm', position_error_norm, ...
    'velocity_norm', velocity_error_norm);
cw_linearization_validation_result.statistics = struct( ...
    'position', position_stats, ...
    'velocity', velocity_stats);
cw_linearization_validation_result.figures = struct( ...
    'trajectory', fig_trajectory, ...
    'position_error', fig_position_error, ...
    'velocity_error', fig_velocity_error);

%% 局部函数
function [rel_r_history, rel_v_history] = generateCwHistory2D(n, rel_r0, rel_v0, time_list)
% generateCwHistory2D 用 CW 状态转移矩阵生成二维相对状态历史。

arguments
    n (1,1) double {mustBePositive}
    rel_r0 (2,1) double
    rel_v0 (2,1) double
    time_list (1,:) double {mustBeReal, mustBeFinite}
end

sample_num = numel(time_list);
rel_r_history = zeros(2, sample_num);
rel_v_history = zeros(2, sample_num);

for i = 1:sample_num
    [Phi_rr, Phi_rv, Phi_vr, Phi_vv] = cw_stm_2d(n, time_list(i));
    rel_r_history(:, i) = Phi_rr * rel_r0 + Phi_rv * rel_v0;
    rel_v_history(:, i) = Phi_vr * rel_r0 + Phi_vv * rel_v0;
end

end

function stats = summarizeErrorHistory(error_history, error_norm_history, time_list)
% summarizeErrorHistory 汇总分量误差与范数误差统计量。

arguments
    error_history (2,:) double
    error_norm_history (1,:) double
    time_list (1,:) double
end

[max_norm, idx_max_norm] = max(error_norm_history);

stats = struct();
stats.max_abs = max(abs(error_history), [], 2);
stats.rms = sqrt(mean(error_history .^ 2, 2));
stats.max_norm = max_norm;
stats.max_norm_time = time_list(idx_max_norm);
stats.final = error_history(:, end);
stats.final_norm = error_norm_history(end);

end
