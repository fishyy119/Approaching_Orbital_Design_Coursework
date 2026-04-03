%% 两脉冲转移最优时间搜索
%
% 接o8，遍历转移时间范围，计算并比较不同时间下的总Δv，找到最优解。

clear; clc;
utils.setDefaultGraphics();

%% 常数
mu = 3.986004418e14; % 地球引力参数
Re = 6378137;
h = 1000e3;
r = Re + h;

n = sqrt(mu / r^3);

%% 初始/终端条件
deg = pi / 180;
dtheta = 1 * deg;

% 初始相对状态（CW坐标）
r0 = [
      r * (cos(dtheta) - 1);
      -r * sin(dtheta);
      0
      ];

v0 = [0; 0; 0];

% 目标：完全交会
rf = [0; 0; 0];
vf = [0; 0; 0];

%% 搜索转移时间范围：2h
T_min = 100; % 避免 T=0
T_max = 7200; % 2小时
T_steps = 300; % 离散步数
T_list = linspace(T_min, T_max, T_steps);

dv1_norm = zeros(size(T_list));
dv2_norm = zeros(size(T_list));
dv_total = zeros(size(T_list));

%% 遍历每个转移时间
for k = 1:length(T_list)
    T = T_list(k);

    % 状态转移矩阵
    [Phi_rr, Phi_rv, Phi_vr, Phi_vv] = cw_stm(n, T);

    % 第一次脉冲后的速度
    v0_plus = Phi_rv \ (rf - Phi_rr * r0);

    % 第二次脉冲前速度
    vf_minus = Phi_vr * r0 + Phi_vv * v0_plus;

    % 两次脉冲
    dv1 = v0_plus - v0;
    dv2 = vf - vf_minus;

    % 存储模长
    dv1_norm(k) = norm(dv1);
    dv2_norm(k) = norm(dv2);
    dv_total(k) = dv1_norm(k) + dv2_norm(k);
end

%% 找最优
[~, idx_opt] = min(dv_total);
T_opt = T_list(idx_opt);
dv_opt = dv_total(idx_opt);

fprintf('最优转移时间 T = %.2f s\n', T_opt);
fprintf('最小总 Δv = %.4f m/s\n', dv_opt);

%% 绘制 Δv 曲线
utils.createFigureA4();
hold on; grid on;
plot(T_list / 60, dv1_norm, 'r', 'LineWidth', 1.5);
plot(T_list / 60, dv2_norm, 'b', 'LineWidth', 1.5);
plot(T_list / 60, dv_total, 'k', 'LineWidth', 2);
xlabel('Transfer Time (min)');
ylabel('Δv (m/s)');
legend('Δv1', 'Δv2', 'Δv_{total}');
title('Two-Impulse Rendezvous: Δv vs Transfer Time');
