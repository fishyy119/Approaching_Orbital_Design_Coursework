%% 多段脉冲抵近仿真
%
% 本脚本用于多段脉冲推力的航天器抵近机动仿真。
% 通过可配置的滑移趋近律实现会合方案设计。
%
% 功能：
%   - 支持配置抵近轨迹：直线路径、抛物线路径
%   - 支持配置趋近律类型：指数型、快速型、慢速型

clear;
clc;
utils.setDefaultGraphics();
%% 参数
sim_case = 1;
N = 5;

mu = 3.986004418e14;
Re = 6378137;

if sim_case == 1
    h = 1000e3;
    r0 = [-5000; -500; 0];
    v0 = [10; 10; 0];
    rf = [-500; -50; 0];
    vf = [1; 1; 0];
    drho0 = -10;
    drhot = -0.5;
    line = "line";
else
    h = 500e3;
    r0 = [6000; -8000; 0];
    v0 = [1.5; -1; 0];
    rf = [60; -80; 0];
    vf = [0.03; -0.04; 0];
    drho0 = -10;
    drhot = -1;
    line = "parabola";
end

r = Re + h;
n = sqrt(mu / r^3);
rho0 = norm(r0 - rf);
rhoT = 0;

types = ["exp", "slow", "fast"];

param = TransferParams;
param.n = n;
param.r0 = r0;
param.v0 = v0;

param.rf = rf;
param.vf = vf;

param.rho0 = rho0;
param.rhoT = rhoT;

param.drho0 = drho0;
param.drhoT = drhot;

param.N = N;
param.path_type = line;

%%
results = struct();

% 绘图
utils.createFigureA4();
hold on;
grid on;

for i = 1:length(types)
    type = types(i);
    param.schedule_type = type;

    [traj_r, DV_sum, T] = solve_transfer(param);

    results.(type).dv = DV_sum;
    results.(type).T = T;

    plot(traj_r(1, :), traj_r(2, :), 'LineWidth', 1.5);
end

legend(types);
title(utils.formatMixedFontText('不同rho调度策略对比'));
% 控制台输出
disp("==== 统计结果 ====");

for i = 1:length(types)
    type = types(i);
    fprintf("%s:  T = %.4f,  总ΔV = %.4f\n", ...
        type, results.(type).T, results.(type).dv);
end

%%
function [traj_r, DV_sum, T] = solve_transfer(param)

arguments
    param(1, 1) TransferParams
end

n = param.n;
r0 = param.r0;
v0 = param.v0;
rf = param.rf;
vf = param.vf;

rho0 = param.rho0;
rhoT = param.rhoT;
drho0 = param.drho0;
drhoT = param.drhoT;

N = param.N;
type = param.schedule_type;

switch type

    case "exp"
        a = (drho0 - drhoT) / rho0;
        T = log(drhoT / drho0) / a;

        dt = T / N;
        t = 0:dt:T;

        rhos = rho0 * exp(a * t) + drhoT * (exp(a * t) - 1) / a;

    case "slow"
        a = (drho0 - drhoT) / rho0^2;
        k = sqrt(a * drhoT);
        T = (atan(a * rhoT / k) - atan(a * rho0 / k)) / k;

        dt = T / N;
        t = 0:dt:T;

        rhos = tan(k * t + atan(a * rho0 / k)) * k / a;

    case "fast"
        rhoStar = rho0;
        a = ((rho0 + rhoStar) * (rhoT + rhoStar) * (drho0 - drhoT)) / (rhoT - rho0);
        b = drho0 - a / (rho0 + rhoStar);

        T = (rhoT - rho0) / b ...
            -a / b^2 * log((a + b * (rhoT + rho0)) / (a + 2 * b * rho0));

        dt = T / N;
        t = 0:dt:T;

        F = @(rho, tt) (rho - rho0) / b ...
            - (a / b^2) * log((a + b * (rho + rho0)) / (a + 2 * b * rho0)) ...
            -tt;

        rhos = zeros(size(t));
        rhos(1) = rho0;

        for i = 2:length(t)
            rhos(i) = fzero(@(rho) F(rho, t(i)), rhos(i - 1));
        end

end

r_seq = gen_waypoints(param, rhos);

dt = T / N;

r_cur = r0;
v_cur = v0;

traj_r = r_cur;
DV = zeros(3, N + 1);

for k = 1:N

    [Phi_rr, Phi_rv, Phi_vr, Phi_vv] = cw_stm(n, dt);

    r_target = r_seq(:, k + 1);

    dv = Phi_rv \ (r_target - Phi_rr * r_cur - Phi_rv * v_cur);

    v_cur = v_cur + dv;

    for ddt = linspace(0, dt, 50)
        [Phi_rr_d, Phi_rv_d, ~, ~] = cw_stm(n, ddt);
        traj_r = [traj_r, Phi_rr_d * r_cur + Phi_rv_d * v_cur];
    end

    r_next = Phi_rr * r_cur + Phi_rv * v_cur;
    v_next = Phi_vr * r_cur + Phi_vv * v_cur;

    r_cur = r_next;
    v_cur = v_next;

    DV(:, k) = dv;
end

DV(:, N + 1) = vf - v_cur;
DV_sum = sum(vecnorm(DV));

end

%%
function r_seq = gen_waypoints(param, rhos)

r0 = param.r0;
rf = param.rf;
type = param.path_type;
N = length(rhos);

r_seq = zeros(3, N);

dr = r0 - rf;
L = norm(dr);

% 单位方向（主方向）
ex = dr / L;

switch type
    case "line"

        for k = 1:N
            rho = rhos(k);
            r_seq(:, k) = rf + rho * ex;
        end

    case "parabola"
        xt = rf(1);
        a = (r0(2) - rf(2)) / (r0(1) - rf(1))^2;

        r_seq(:, 1) = r0;

        for k = 2:N
            rho = rhos(k);
            coeff = [; ...
                         a^2, ...
                         -4 * a^2 * xt, ...
                         (6 * a^2 * xt^2 + 1), ...
                         - (4 * a^2 * xt^3 + 2 * xt), ...
                         (a^2 * xt^4 + xt^2 + rf(3)^2 - rho^2); ...
                     ];
            roots_all = roots(coeff);
            roots_real = roots_all(abs(imag(roots_all)) < 1e-8);
            roots_real = real(roots_real);

            if isempty(roots_real)
                error("No valid root found");
            end

            [~, idx] = min(abs(roots_real - r_seq(1, k - 1)));
            xm = roots_real(idx);
            ym = a * (xm - xt)^2 + rf(2);

            r_seq(:, k) = [xm; ym; 0];
        end

end

end
