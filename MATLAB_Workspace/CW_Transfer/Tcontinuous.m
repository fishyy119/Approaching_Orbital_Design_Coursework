%% 连续推力LQR控制
%
% 本脚本使用线性二次调节器(LQR)设计连续推力控制律。
% 基于Clohessy-Wiltshire方程，实现航天器的相对轨道控制。
%
% 功能：
%   - 设计LQR控制律

clear;
clc;
utils.setDefaultGraphics();

%% 参数定义
mu = 3.986004418e14; % 地球引力参数
Re = 6378137; % 地球半径
h1 = 1000e3; % 航天器1高度
h2 = 990e3; % 航天器2高度
r1 = Re + h1;
r2 = Re + h2;
n1 = sqrt(mu / r1^3);
n2 = sqrt(mu / r2^3); % 轨道角速度
%% 状态空间矩阵 (CW方程)
A = [0, 0, 0, 1, 0, 0; ...
         0, 0, 0, 0, 1, 0; ...
         0, 0, 0, 0, 0, 1; ...
         3 * n2^2, 0, 0, 0, 2 * n2, 0; ...
         0, 0, 0, -2 * n2, 0, 0; ...
         0, 0, -n2^2, 0, 0, 0];

B = [0, 0, 0; ...
         0, 0, 0; ...
         0, 0, 0; ...
         1, 0, 0; ...
         0, 1, 0; ...
         0, 0, 1];
%% LQR权重
Q = diag([1, 1, 1, 0.1, 0.1, 0.1]);
R = 2e10 * eye(3);
%% 求解Riccati方程
K = lqr(A, B, Q, R);
%% 时间离散化
dt = 1;
T = 2000;
time = 0:dt:T;
N = length(time);
%% 参考轨道 (航天器2圆轨道)
N = length(time);
x_t = zeros(3, N);
xdot_t = zeros(3, N);
xddot_t = zeros(3, N);

e0_r = [r2 - r1; 0; 0];
e0_v = [0; r2 * n2 - r1 * n1; 0];
e0 = [e0_r; e0_v]; % 6x1

for k = 1:N
    t = time(k);
    [Phi_rr, Phi_rv, Phi_vr, Phi_vv] = cw_stm(n2, t);
    Phi = [Phi_rr, Phi_rv; Phi_vr, Phi_vv]; % 6x6

    e_t = Phi * e0; % 6x1
    x_t(:, k) = e_t(1:3);
    xdot_t(:, k) = e_t(4:6);
    xddot_t(:, k) = A(4:6, :) * e_t;
end

%% 仿真
x = [0; 0; 0; 0; 0; 0];
xt = [r2 - r1; 0; 0; 0; r2 * n2 - r1 * n1; 0];
e = x - xt;
x_hist = zeros(6, N);
u_hist = zeros(3, N);

for k = 1:N
    xt = [x_t(1, k); x_t(2, k); x_t(3, k); xdot_t(1, k); xdot_t(2, k); xdot_t(3, k)];
    ut = [xddot_t(1, k); xddot_t(2, k); xddot_t(3, k)]; % 参考加速度
    uhat = -K * e;
    u = uhat - [3 * n1^2 * x_t(1, k); 0; -n1^2 * x_t(3, k)];
    u = uhat; % 考虑目标无控运动后，拖着的尾巴刚好抵消

    % 连续欧拉积分
    x_dot = A * x + B * u;
    x = x + x_dot * dt;
    e = x - xt;

    % e_dot = A * e + B * uhat;
    % e = e + e_dot * dt;

    x_hist(:, k) = e + xt;
    u_hist(:, k) = u;
end

%% 绘图
utils.createFigureA4();
plot(x_hist(1, :), x_hist(2, :), 'b', 'LineWidth', 2);
hold on;
plot(x_t(1, :), x_t(2, :), 'r--', 'LineWidth', 1.5);
legend(utils.formatMixedFontText({'航天器1轨迹', '航天器2轨迹'}));
xlabel('X');
ylabel('Y');
grid on;

utils.createFigureA4();
plot(time, u_hist(1:2, :), 'LineWidth', 2);
legend('u_x', 'u_y');
xlabel('Time (s)');
ylabel('a');
grid on;
