%% 轨道转移3
%
% 计算单次脉冲改变近地点幅角所需的速度增量。

clear
clc
%% 常数
mu = 398600; % km^3/s^2
Re = 6378; % km
%% 轨道参数
hp = 400; % 近地点高度 km
ha = 600; % 远地点高度 km

rp = Re + hp;
ra = Re + ha;

a = (rp + ra) / 2;
e = (ra - rp) / (ra + rp);
p = a * (1 - e^2);
%% 近地点辐角变化
domega = deg2rad(60);
%% 两个交点真近点角
f1 = domega / 2;
f2 = f1 + pi;

f = [f1, f2];
%% 计算函数
state = @(f, omega) orbit_state(f, omega, p, e, mu);
%% 计算两交点Δv
dv = zeros(1, 2);

for i = 1:2

    fi = f(i);

    % 原轨道
    [r1, v1] = state(fi, 0);

    % 新轨道
    [r2, v2] = state(fi - domega, domega);

    dv(i) = norm(v2 - v1);

    fprintf('交点 %d\n', i)
    fprintf('r = %.2f km\n', norm(r1))
    fprintf('dv = %.5f km/s\n\n', dv(i))

end

%% 绘图

% 近地点辐角
omega1 = 0;
omega2 = deg2rad(60);

% 真近点角
f = linspace(0, 2 * pi, 1000);

% 轨道1
r1 = p ./ (1 + e * cos(f));
x1 = r1 .* cos(f + omega1);
y1 = r1 .* sin(f + omega1);

% 轨道2
r2 = p ./ (1 + e * cos(f));
x2 = r2 .* cos(f + omega2);
y2 = r2 .* sin(f + omega2);

figure
hold on
axis equal
grid on

plot(x1, y1, 'b', 'LineWidth', 2)
plot(x2, y2, 'g', 'LineWidth', 2)

plot(0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r')

xlabel('x (km)')
ylabel('y (km)')

legend('Original Orbit', 'Rotated Orbit', 'Earth')

function [r_vec, v_vec] = orbit_state(f, omega, p, e, mu)

r = p / (1 + e * cos(f));

% PQW坐标
r_pqw = [r * cos(f); r * sin(f); 0];

v_pqw = sqrt(mu / p) * [-sin(f); e + cos(f); 0];

% 旋转矩阵
R = [cos(omega), -sin(omega), 0; ...
         sin(omega), cos(omega), 0; ...
         0, 0, 1];

r_vec = R * r_pqw;
v_vec = R * v_pqw;

end
