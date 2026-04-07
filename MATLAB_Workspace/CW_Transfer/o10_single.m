%% 多段脉冲转移仿真
%
% 本脚本进行多段脉冲推力的轨道转移仿真。
% 使用简单的等比数列生成中间脉冲点

clear;
clc;
utils.setDefaultGraphics();

%% 常数
mu = 3.986004418e14;
Re = 6378137;
h = 1000e3;
r = Re + h;

n = sqrt(mu / r^3);

%% 初始 / 终端条件
r0 = [-5000; -500; 0];
v0 = [10; 10; 0];

rf = [-500; -50; 0];
vf = [1; 1; 0];

T = 3600; % 1h
N = 2; % 段数（可调）
q = 0.4; % 公比
dt = T / N;

[r_seq, v_seq] = gen_geometric_waypoints(r0, v0, rf, vf, N, q);

%% 初始化
r_cur = r0;
v_cur = v0;

DV = zeros(3, N + 1); % 记录各次脉冲速度矢量

traj_r = r_cur;

%% 多段推进
for k = 1:N

    % STM
    [Phi_rr, Phi_rv, Phi_vr, Phi_vv] = cw_stm(n, dt);

    % 目标点
    r_target = r_seq(:, k + 1);

    % 计算 Δv
    dv = Phi_rv \ (r_target - Phi_rr * r_cur - Phi_rv * v_cur);

    % 更新速度（施加脉冲）
    v_cur = v_cur + dv;

    % 推进
    r_next = Phi_rr * r_cur + Phi_rv * v_cur;
    v_next = Phi_vr * r_cur + Phi_vv * v_cur;

    % 更新
    r_cur = r_next;
    v_cur = v_next;

    DV(:, k) = dv;
    traj_r = [traj_r, r_cur];

end

DV(:, N + 1) = vf - v_cur;

%% 输出
disp("总ΔV：");
disp(sum(vecnorm(DV, 2, 1)));

plot_trajectory_cw(traj_r, r_seq, n, dt);
%%

function [r_seq, v_seq] = gen_geometric_waypoints(r0, v0, rf, vf, N, q)

r_seq = zeros(3, N + 1);
v_seq = zeros(3, N + 1);

for k = 0:N

    if abs(q - 1) < 1e-8
        alpha = k / N; % 等距（退化情况）
    else
        alpha = (q^k - 1) / (q^N - 1); % 等比
    end

    r_seq(:, k + 1) = r0 + alpha * (rf - r0);
    v_seq(:, k + 1) = v0 + alpha * (vf - v0);
end

end

%%

function plot_trajectory_cw(traj_r, r_seq, n, dt)

utils.createFigureA4();
hold on;
grid on;
axis equal;

N = size(traj_r, 2) - 1; % 段数
sub_steps = 50; % 每段细分（可调）

traj_dense = [];

for k = 1:N

    % 当前段起点状态
    r0 = traj_r(:, k);
    r1 = traj_r(:, k + 1);

    % ⚠️ 关键：需要恢复该段初始速度
    % 用差分反推 v（仅用于绘图，不影响主逻辑）
    % 利用：r1 = Phi_rr*r0 + Phi_rv*v0
    [Phi_rr, Phi_rv, ~, ~] = cw_stm(n, dt);
    v0 = Phi_rv \ (r1 - Phi_rr * r0);

    % 段内细分传播
    for i = 0:sub_steps
        tau = dt * i / sub_steps;

        [Phi_rr_s, Phi_rv_s, ~, ~] = cw_stm(n, tau);
        r_s = Phi_rr_s * r0 + Phi_rv_s * v0;

        traj_dense = [traj_dense, r_s];
    end

end

plot3(traj_dense(1, :), traj_dense(2, :), traj_dense(3, :), ...
    'b-', 'LineWidth', 1.5);

plot3(r_seq(1, :), r_seq(2, :), r_seq(3, :), ...
    'ro--', 'LineWidth', 1);

scatter3(traj_r(1, 1), traj_r(2, 1), traj_r(3, 1), 60, 'g', 'filled');
scatter3(traj_r(1, end), traj_r(2, end), traj_r(3, end), 60, 'k', 'filled');

xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');

legend(utils.formatMixedFontText({'轨迹', '中间点（直线插值）', '起点', '终点'}));

title(utils.formatMixedFontText('CW多脉冲分段转移轨迹'));

end
