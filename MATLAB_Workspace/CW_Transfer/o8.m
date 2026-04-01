%% CW方程两脉冲转移2
%
% 本脚本使用Clohessy-Wiltshire方程计算不同转移时间下的两脉冲抵近转移。
% 与o7类似，不过其初值需要额外推导（同一圆轨道上真近点角相差1度的飞行器）

clear;
clc;
%% 常数
mu = 3.986e14; % 地球引力常数
Re = 6371e3;
h = 1000e3;
r = Re + h;

n = sqrt(mu / r^3);
%% 初始/终端条件
deg = pi / 180;
dtheta = 1 * deg;

% ===== 目标航天器（惯性系）=====
rt = [r; 0; 0];
vt = [0; r * n; 0];

% ===== 旋转矩阵（滞后 → 负角度）=====
Rz = [; ...
          cos(-dtheta), -sin(-dtheta), 0; ...
          sin(-dtheta), cos(-dtheta), 0; ...
          0, 0, 1; ...
      ];

% ===== 追踪航天器（惯性系）=====
rc = Rz * rt;
vc = Rz * vt;

% ===== 相对状态（惯性系）=====
dr = rc - rt;
dv = vc - vt;

% ===== 构造 LVLH 基 =====
ex = rt / norm(rt); % 径向
ez = cross(rt, vt);
ez = ez / norm(ez); % 法向
ey = cross(ez, ex); % 切向

R_I2L = [ex'; ey'; ez']; % 惯性 → LVLH

% ===== 转换到 LVLH =====\
omega = [0; 0; n];
r0 = R_I2L * dr;
v0 = R_I2L * dv - cross(omega, r0); % 坐标系旋转的牵连项，抵消为0

% 另一种直接看出来的状态初值
% r0 = [
%     r * (cos(dtheta) - 1);
%    -r * sin(dtheta);
%     0
% ];
%
% v0 = [0; 0; 0];

% 目标：完全交会
rf = [0; 0; 0];
vf = [0; 0; 0];

times = [1000, 2000, 3000];
%% 数据存储
results = struct();

for k = 1:length(times)
    T = times(k);

    % 状态转移矩阵
    [Phi_rr, Phi_rv, Phi_vr, Phi_vv] = cw_stm(n, T);

    % 第一次脉冲后的速度
    v0_plus = Phi_rv \ (rf - Phi_rr * r0);

    % 第二次脉冲前速度
    vf_minus = Phi_vr * r0 + Phi_vv * v0_plus;

    % 两次脉冲
    dv1 = v0_plus - v0;
    dv2 = vf - vf_minus;

    % 存储
    results(k).T = T;
    results(k).dv1 = dv1;
    results(k).dv2 = dv2;
    results(k).dv_total = norm(dv1) + norm(dv2);
    results(k).v0_plus = v0_plus;

    fprintf("T = %d s\n", T);
    fprintf("Δv1 = [%f %f %f]\n", dv1);
    fprintf("Δv2 = [%f %f %f]\n", dv2);
    fprintf("Total Δv = %f m/s\n\n", results(k).dv_total);
end

figure;
hold on;
grid on;
colors = ['r', 'g', 'b'];

for k = 1:length(results)
    T = results(k).T;
    traj = generate_trajectory(n, r0, results(k).v0_plus, T, 200);

    plot(traj(1, :), traj(2, :), colors(k), 'LineWidth', 1.5);
end

xlabel('x (m)');
ylabel('y (m)');
legend('T=1000s', 'T=2000s', 'T=3000s');
title('CW Transfer Trajectories');
axis equal;
%%

function traj = generate_trajectory(n, r0, v0_plus, T, steps)

tspan = linspace(0, T, steps);
traj = zeros(3, steps);

for i = 1:steps
    t = tspan(i);
    [Phi_rr, Phi_rv, ~, ~] = cw_stm(n, t);

    r = Phi_rr * r0 + Phi_rv * v0_plus;
    traj(:, i) = r;
end

end
