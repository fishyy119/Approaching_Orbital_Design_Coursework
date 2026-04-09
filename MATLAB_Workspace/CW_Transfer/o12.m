%% 水滴悬停轨道设计仿真
%
% 依据课件第 116-119 页附近的水滴悬停公式，设计满足周期闭合条件的
% 平面 CW 相对轨迹，并对不同悬停周期比 alpha 做参数扫描。

clear;
clc;
utils.setDefaultGraphics();

%% 目标圆轨道参数
mu = 3.986004418e14; % 地球引力参数
a_target = 42250e3; % 目标圆轨道半长轴

n = sqrt(mu / a_target^3);
Tc = 2 * pi / n;

%% 设计输入
r0 = [-1; -0.1; 0] * 1e3; % 课件示例初始相对位置
alpha_design = 0.1; % 单个设计示例
alpha_list = [0.05, 0.1, 0.2, 0.5]; % 课件示例扫描点

sample_count = 2000; % 每个水滴周期的采样点数

%% 单个水滴设计
design = design_hover_drop(n, r0, alpha_design);
traj_design = generate_cw_trajectory( ...
    n, r0, design.v0_req, design.T, sample_count);

[x_span_design, y_span_design] = compute_xy_span(traj_design);

%% 输出单个设计结果
disp("==== 水滴悬停单个设计 ====");
fprintf('目标轨道周期 Tc = %.2f s\n', Tc);
fprintf('悬停周期比 alpha = %.3f\n', alpha_design);
fprintf('水滴周期 T = %.2f s\n', design.T);
fprintf('初始相对位置 r0 = [%.3f %.3f %.3f] km\n', r0 / 1e3);
fprintf('课件闭式解 v0 = [%.5f %.5f %.5f] m/s\n', design.v0_req);
fprintf('STM 反解 v0 = [%.5f %.5f %.5f] m/s\n', design.v0_from_stm);
fprintf('速度解差异范数 = %.3e m/s\n', design.velocity_consistency_error);
fprintf('周期末位置闭合误差 = %.3e m\n', design.position_closure_error);
fprintf('周期末速度 v(T-) = [%.5f %.5f %.5f] m/s\n', design.vT_minus);
fprintf('重置脉冲 Delta v = [%.5f %.5f %.5f] m/s\n', design.dv_reset);
fprintf('课件脉冲公式误差 = %.3e m/s\n', design.dv_formula_error);
fprintf('单次机动速度增量 = %.5f m/s\n', norm(design.dv_reset));
fprintf('一个目标轨道周期机动次数 = %.2f\n', design.impulses_per_target_orbit);
fprintf('一个目标轨道周期总速度增量 = %.5f m/s\n', ...
    design.total_dv_per_target_orbit);
fprintf('水滴 x 向跨度 = %.4f km\n', x_span_design / 1e3);
fprintf('水滴 y 向跨度 = %.4f km\n', y_span_design / 1e3);

%% 周期比扫描
scan_stats = scan_hover_drop_cases(n, r0, alpha_list, sample_count);

disp(" ");
disp("==== 悬停周期比扫描结果 ====");
print_scan_summary(scan_stats);

%% 绘图：单个水滴轨迹
mid_idx = round(size(traj_design, 2) / 2);

utils.createFigureA4();
hold on;
grid on;
axis equal;

h_target = scatter(0, 0, 70, 'k', 'filled');
h_traj = plot(traj_design(1, :) / 1e3, traj_design(2, :) / 1e3, ...
    'r-', 'LineWidth', 1.8);
h_start = scatter(r0(1) / 1e3, r0(2) / 1e3, 60, 'b', 'filled');
h_mid = scatter(traj_design(1, mid_idx) / 1e3, traj_design(2, mid_idx) / 1e3, ...
    55, 'm', 'filled');

xlabel('x (km)');
ylabel('y (km)');
legend([h_target, h_traj, h_start, h_mid], ...
    utils.formatMixedFontText({'目标飞行器', '水滴悬停轨迹', '起点 / 终点', '半周期点'}), ...
    'Location', 'best');
utils.applyPlotTitle(gca, utils.formatMixedFontText(sprintf('水滴悬停轨迹，alpha = %.2f', alpha_design)));

%% 绘图：周期比扫描结果
utils.createFigureA4();
subplot(2, 2, 1);
plot(scan_stats.alpha, scan_stats.single_dv, 'o-', 'LineWidth', 1.5);
grid on;
xlabel('\alpha');
ylabel('Single-cycle \Delta{\itv} (m/s)');
utils.applyPlotTitle(gca, utils.formatMixedFontText('单次机动速度增量'));

subplot(2, 2, 2);
plot(scan_stats.alpha, scan_stats.total_dv_per_target_orbit, ...
    'o-', 'LineWidth', 1.5);
grid on;
xlabel('\alpha');
ylabel('Total \Delta{\itv} per target orbit (m/s)');
utils.applyPlotTitle(gca, utils.formatMixedFontText('目标轨道周期总速度增量'));

subplot(2, 2, 3);
plot(scan_stats.alpha, scan_stats.x_span_km, 'o-', 'LineWidth', 1.5);
grid on;
xlabel('\alpha');
ylabel('x span (km)');
utils.applyPlotTitle(gca, utils.formatMixedFontText('水滴 x 向跨度'));

subplot(2, 2, 4);
plot(scan_stats.alpha, scan_stats.y_span_km, 'o-', 'LineWidth', 1.5);
grid on;
xlabel('\alpha');
ylabel('y span (km)');
utils.applyPlotTitle(gca, utils.formatMixedFontText('水滴 y 向跨度'));

%% 局部函数
function design = design_hover_drop(n, r0, alpha)

arguments
    n(1, 1) double{mustBePositive}
    r0(3, 1) double
    alpha(1, 1) double{mustBePositive, mustBeLessThan(alpha, 1)}
end

if abs(r0(3)) > 1e-12
    error('当前脚本仅考虑平面水滴悬停轨道，请设置 z0 = 0。');
end

T = 2 * pi * alpha / n;
beta = alpha * pi;
s = sin(beta);
c = cos(beta);

den = 4 * s - 3 * beta * c;
if abs(den) < 1e-10
    error('当前 alpha 使水滴闭合速度公式奇异，请更换 alpha。');
end

x0 = r0(1);

xdot0 = -(3 * beta * s / den) * n * x0;
ydot0 = (6 * (beta * c - s) / den) * n * x0;
v0_req = [xdot0; ydot0; 0];

[Phi_rr, Phi_rv, Phi_vr, Phi_vv] = cw_stm(n, T);

Phi_rv_xy = Phi_rv(1:2, 1:2);
rhs_xy = ((eye(3) - Phi_rr) * r0);
rhs_xy = rhs_xy(1:2);

if rcond(Phi_rv_xy) < 1e-10
    error('当前 alpha 对应的平面 Phi_rv 子块近似奇异，无法稳定反解初始速度。');
end

v0_from_stm_xy = Phi_rv_xy \ rhs_xy;
v0_from_stm = [v0_from_stm_xy; 0];

rT = Phi_rr * r0 + Phi_rv * v0_req;
vT_minus = Phi_vr * r0 + Phi_vv * v0_req;

dv_reset = v0_req - vT_minus;
dv_formula = [2 * xdot0; 0; 0];

design = struct();
design.alpha = alpha;
design.T = T;
design.v0_req = v0_req;
design.v0_from_stm = v0_from_stm;
design.rT = rT;
design.vT_minus = vT_minus;
design.dv_reset = dv_reset;
design.velocity_consistency_error = norm(v0_req - v0_from_stm);
design.position_closure_error = norm(rT - r0);
design.dv_formula_error = norm(dv_reset - dv_formula);
design.impulses_per_target_orbit = 1 / alpha;
design.total_dv_per_target_orbit = norm(dv_reset) / alpha;

end

function scan_stats = scan_hover_drop_cases(n, r0, alpha_list, sample_count)

arguments
    n(1, 1) double{mustBePositive}
    r0(3, 1) double
    alpha_list(1, :) double{mustBePositive}
    sample_count(1, 1) double{mustBeInteger, mustBePositive}
end

case_num = numel(alpha_list);

single_dv = zeros(case_num, 1);
impulses_per_orbit = zeros(case_num, 1);
total_dv_per_orbit = zeros(case_num, 1);
x_span_km = zeros(case_num, 1);
y_span_km = zeros(case_num, 1);

for i = 1:case_num
    design = design_hover_drop(n, r0, alpha_list(i));
    traj = generate_cw_trajectory(n, r0, design.v0_req, design.T, sample_count);
    [x_span, y_span] = compute_xy_span(traj);

    single_dv(i) = norm(design.dv_reset);
    impulses_per_orbit(i) = design.impulses_per_target_orbit;
    total_dv_per_orbit(i) = design.total_dv_per_target_orbit;
    x_span_km(i) = x_span / 1e3;
    y_span_km(i) = y_span / 1e3;
end

scan_stats = struct();
scan_stats.alpha = alpha_list(:);
scan_stats.single_dv = single_dv;
scan_stats.impulses_per_orbit = impulses_per_orbit;
scan_stats.total_dv_per_target_orbit = total_dv_per_orbit;
scan_stats.x_span_km = x_span_km;
scan_stats.y_span_km = y_span_km;

end

function traj = generate_cw_trajectory(n, r0, v0, T, sample_count)

arguments
    n(1, 1) double{mustBePositive}
    r0(3, 1) double
    v0(3, 1) double
    T(1, 1) double{mustBePositive}
    sample_count(1, 1) double{mustBeInteger, mustBePositive}
end

t_list = linspace(0, T, sample_count);
traj = zeros(3, sample_count);

for i = 1:sample_count
    [Phi_rr, Phi_rv, ~, ~] = cw_stm(n, t_list(i));
    traj(:, i) = Phi_rr * r0 + Phi_rv * v0;
end

end

function [x_span, y_span] = compute_xy_span(traj)

x_span = max(traj(1, :)) - min(traj(1, :));
y_span = max(traj(2, :)) - min(traj(2, :));

end

function print_scan_summary(scan_stats)

fprintf('%8s %14s %14s %18s %12s %12s\n', ...
    'alpha', 'single_dv', 'impulses', 'total_dv_per_orbit', 'x_span', 'y_span');

for i = 1:numel(scan_stats.alpha)
    fprintf('%8.2f %14.4f %14.2f %18.4f %12.4f %12.4f\n', ...
        scan_stats.alpha(i), scan_stats.single_dv(i), ...
        scan_stats.impulses_per_orbit(i), ...
        scan_stats.total_dv_per_target_orbit(i), ...
        scan_stats.x_span_km(i), scan_stats.y_span_km(i));
end

end
