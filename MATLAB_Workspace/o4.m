clc;
clear;
%% 已知参数
Re = 6378; % km
mu = 398600; % km^3/s^2

r1 = 1545 + Re; % km
r2 = 852 + Re; % km
f1 = deg2rad(126);
f2 = deg2rad(58);
%% 求解 e 与 p
A = [r1 * cos(f1), -1; ...
    r2 * cos(f2), -1];

b = [-r1; -r2];

x = A \ b;

e = x(1);
p = x(2);
%% 半长轴
a = p / (1 - e^2);
%% 近地点距离
rp = a * (1 - e);
%% 近地点高度 (地球半径6378km)
hp = rp - Re;
%% 轨道周期
T = 2 * pi * sqrt(a^3/mu);
%% 最大最小速度
ra = a * (1 + e);

v_max = sqrt(mu*(2 / rp - 1 / a));
v_min = sqrt(mu*(2 / ra - 1 / a));
%% 两点弦长
c = sqrt(r1^2+r2^2-2*r1*r2*cos(f1-f2));
%% 输出
fprintf("偏心率 e = %.4f\n", e)
fprintf("近地点高度 hp = %.2f km\n", hp)
fprintf("半长轴 a = %.2f km\n", a)
fprintf("周期 T = %.2f s (%.2f min)\n", T, T/60)
fprintf("最大速度 = %.3f km/s\n", v_max)
fprintf("最小速度 = %.3f km/s\n", v_min)
fprintf("两点弦长 = %.2f km\n", c)
%% 绘图

f = linspace(0, 2*pi, 1000);
r = p ./ (1 + e * cos(f));

x = r .* cos(f);
y = r .* sin(f);

xo = Re .* cos(f);
yo = Re .* sin(f);

x1 = r1 * cos(f1);
y1 = r1 * sin(f1);

x2 = r2 * cos(f2);
y2 = r2 * sin(f2);


figure
plot(x, y, 'b', 'LineWidth', 2)
hold on

fill(xo, yo, 'g', 'EdgeColor', 'k')

plot(x1, y1, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r')
plot(x2, y2, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g')

text(x1, y1, '  Point 1')
text(x2, y2, '  Point 2')

axis equal
grid on

xlabel('x (km)')
ylabel('y (km)')
title('Elliptical Orbit')
