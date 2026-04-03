%% 霍曼转移交会仿真
%
% 仿真霍曼转移轨道交会问题。
% 给定允许交会误差，计算交会所需的等待时间和转移参数，动画显示追踪器与目标的相对运动。

clc;
clear;
utils.setDefaultGraphics();

transfer = 2;
%% 常量
mu = 3.986004418e14; % 地球引力参数 m^3/s^2
Re = 6378137; % 地球半径 m
tol = 100e3; % 允许交会误差 m
%% 轨道半径
rp = Re + 400e3; % 初始近地点
ra = Re + 600e3; % 初始远地点
rt = Re + 1000e3; % 目标圆轨道
%% 初始轨道参数
a0 = (rp + ra) / 2;
e0 = (ra - rp) / (ra + rp);
%% 平均角速度
nc = sqrt(mu / a0^3); % 初始卫星
nt = sqrt(mu / rt^3); % 目标卫星
%% 霍曼转移参数
if transfer == 1
    aT = (rp + rt) / 2;
    eT = (rt - rp) / (rt + rp);
else
    eT = (rt - ra) / (rt + ra);
    aT = (ra + rt) / 2;
end

tT = pi * sqrt(aT^3 / mu); % 半圈飞行时间
%% 1 交会时允许目标真近点角范围
dtheta = acos((2 * rt^2 - tol^2) / (2 * rt^2)); % 或近似 dtheta = tol/rt

if transfer == 1
    theta_f_min = pi - dtheta;
    theta_f_max = pi + dtheta;
else
    theta_f_min = -dtheta;
    theta_f_max = dtheta;
end

%% 2 倒推机动时刻目标允许角度
theta0_min = theta_f_min - nt * tT;
theta0_max = theta_f_max - nt * tT;
%% 初始角度
theta_c0 = 0; % 追踪器初始真近点角
theta_t0 = pi; % 目标初始真近点角
%% 3 遍历拱点计算等待时间
Tc = 2 * pi / nc; % 追踪器轨道周期

if transfer == 1
    t_a0 = 0; % 第一次到达远地点（拱点）时间
else
    t_a0 = pi / nc; % 近地点机动？
end

t_wait = NaN;

max_iter = 1000; % 最大搜索拱点次数

for k = 0:max_iter
    tw = t_a0 + k * Tc; % 第k次到达拱点时间

    % 该时刻目标角度
    theta_t = theta_t0 + nt * tw;

    % 归一化到 [-pi, pi]
    theta_t = mod(theta_t + pi, 2 * pi) - pi;

    % 检查是否在允许角度范围
    if theta_t >= theta0_min && theta_t <= theta0_max
        t_wait = tw;
        break
    end

end

if isnan(t_wait)
    error('没有找到满足条件的等待时间，请检查参数或增加 max_iter');
end

%% 输出
result.wait_time = t_wait;
result.transfer_time = tT;
result.theta_window = [theta_f_min, theta_f_max];
result.rp = rp;
result.ra = ra;
result.rt = rt;
result.a0 = a0;
result.e0 = e0;
result.aT = aT;

res = result

t_wait = res.wait_time;
tT = res.transfer_time;

rp = res.rp;
ra = res.ra;
rt = res.rt;
%% 时间序列
t_end = t_wait + tT + 2000;
% N = 50000;
% t = linspace(0, t_end, N);
N = 1000;
t = linspace(t_wait - 9000, t_end, N);

rc = zeros(N, 2);
rtg = zeros(N, 2);
%% 初始角度
theta_c0 = 0;
theta_t0 = pi;

nc = sqrt(mu / a0^3);
nt = sqrt(mu / rt^3);

for i = 1:N

    ti = t(i);
    %% 目标轨道（始终圆轨道）
    theta_t = theta_t0 + nt * ti;

    rtg(i, 1) = rt * cos(theta_t);
    rtg(i, 2) = rt * sin(theta_t);
    %% 追踪卫星
    if ti < t_wait
        % 初始椭圆
        theta_c = theta_c0 + nc * ti;

        r = a0 * (1 - e0^2) / (1 + e0 * cos(theta_c));

    elseif ti < t_wait + tT
        % 转移轨道
        tau = ti - t_wait;

        nT = sqrt(mu / aT^3);

        M = nT * tau;

        E = kepler(M, eT);

        theta = 2 * atan2(sqrt(1 + eT) * sin(E / 2), ...
            sqrt(1 - eT) * cos(E / 2));

        r = aT * (1 - eT * cos(E));

        if transfer == 1
            theta_c = theta; % 从拱点出发
        else
            theta_c = theta + pi;
        end

    else
        % 已进入目标轨道
        tau = ti - (t_wait + tT);

        theta_c = theta_t0 + nt * (ti);

        r = rt;

    end

    rc(i, 1) = r * cos(theta_c);
    rc(i, 2) = r * sin(theta_c);

end

%% 绘图

utils.createFigureA4();
hold on
axis equal
grid on

% 轨迹
traj_c = animatedline('Color', 'b', 'LineWidth', 1.5);
traj_t = animatedline('Color', 'r', 'LineWidth', 1.5);

% 当前卫星位置
sat_c = plot(NaN, NaN, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
sat_t = plot(NaN, NaN, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

xlabel('m')
ylabel('m')
title('Rendezvous Animation')

xlim([-1.2e7, 1.2e7])
ylim([-1.2e7, 1.2e7])

%% 逐帧播放

for i = 1:N

    % 更新轨迹
    addpoints(traj_c, rc(i, 1), rc(i, 2));
    addpoints(traj_t, rtg(i, 1), rtg(i, 2));

    % 更新当前点
    sat_c.XData = rc(i, 1);
    sat_c.YData = rc(i, 2);

    sat_t.XData = rtg(i, 1);
    sat_t.YData = rtg(i, 2);

    drawnow

end

function E = kepler(M, e)
E = M;

for k = 1:10
    E = E - (E - e * sin(E) - M) / (1 - e * cos(E));
end

end
