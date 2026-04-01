%% 轨道运动f和g函数
%
% 本脚本计算Lagrange f和g函数及其导数。
% 描述椭圆轨道中真近点角变化过程中的轨道状态变化规律。
%
% 功能：
%   - 计算f、g函数及其导数

clear; clc;

%% 常数
mu = 398600; % km^3/s^2
Re = 6378; % km

%% 轨道参数
rp = Re + 400;
ra = Re + 600;

a = (rp + ra) / 2;
e = (ra - rp) / (ra + rp);
p = a * (1 - e^2);

h = sqrt(mu * p);

%% 初始真近点角
theta0 = 0; % 可设为近地点

r0 = p / (1 + e * cos(theta0));

%% 自变量 Δθ
dtheta = linspace(0, 2 * pi, 500);

%% 预分配
f = zeros(size(dtheta));
g = zeros(size(dtheta));
fdot = zeros(size(dtheta));
fdot2 = zeros(size(dtheta));
gdot = zeros(size(dtheta));

%% 计算
for i = 1:length(dtheta)
    dt = dtheta(i);

    theta = theta0 + dt;
    r = p / (1 + e * cos(theta));

    f(i) = 1 - r / p * (1 - cos(dt));
    g(i) = r * r0 / sqrt(mu * p) * sin(dt);

    fdot(i) = -sqrt(mu) / (r * r0) * sin(dt);
    fdot2(i) = mu / h * (1 - cos(dt)) / sin(dt) * (mu / h^2 * (1 - cos(dt)) - 1 / r0 - 1 / r);
    gdot(i) = 1 - r0 / p * (1 - cos(dt));
end

%% 绘图
figure;

subplot(2, 2, 1);
plot(dtheta, f, 'LineWidth', 1.5);
title('f vs \Delta\theta');
xlabel('\Delta\theta (rad)'); ylabel('f');

subplot(2, 2, 2);
plot(dtheta, g, 'LineWidth', 1.5);
title('g vs \Delta\theta');
xlabel('\Delta\theta (rad)'); ylabel('g');

subplot(2, 2, 3);
plot(dtheta, fdot, 'LineWidth', 1.5);
% plot(dtheta, fdot2, 'LineWidth', 1.5);
title('f dot vs \Delta\theta');
xlabel('\Delta\theta (rad)'); ylabel('fdot');

subplot(2, 2, 4);
plot(dtheta, gdot, 'LineWidth', 1.5);
title('g dot vs \Delta\theta');
xlabel('\Delta\theta (rad)'); ylabel('gdot');
