clear;
clc;
%% 常数
mu = 3.986e14;
Re = 6371e3;
h = 1000e3;
h = 500e3;
r = Re + h;
n = sqrt(mu / r^3);
%% 边界条件
r0 = [-5000; -500; 0];
v0 = [10; 10; 0];
rf = [-500; -50; 0];
vf = [1; 1; 0];
drho0 = -10;
drhot = -0.5;

%
r0 = [6000; -8000; 0];
v0 = [1.5; -1; 0];
rf = [60; -80; 0];
vf = [0.03; -0.04; 0];
drho0 = -10;
drhot = -1;

rho0 = norm(r0-rf);
rhoT = 0;

N = 5;

types = ["exp", "slow", "fast"];
line = "parabola";
% line = "line";

results = struct();

% 绘图
figure;
hold on;
grid on;

for i = 1:length(types)
    type = types(i);

    [traj_r, DV_sum, T] = solve_transfer( ...
        type, line, n, r0, v0, rf, vf, ...
        rho0, rhoT, drho0, drhot, N);

    results.(type).dv = DV_sum;
    results.(type).T = T;

    plot3(traj_r(1, :), traj_r(2, :), traj_r(3, :), 'LineWidth', 1.5);
end

legend(types);
title('不同rho调度策略对比');
% 控制台输出
disp("==== 统计结果 ====");
for i = 1:length(types)
    type = types(i);
    fprintf("%s:  T = %.4f,  总ΔV = %.4f\n", ...
        type, results.(type).T, results.(type).dv);
end

%%
function [traj_r, DV_sum, T] = solve_transfer( ...
    type, line, n, r0, v0, rf, vf, ...
    rho0, rhoT, drho0, drhot, N)

switch type

    case "exp"
        a = (drho0 - drhot) / rho0;
        T = log(drhot/drho0) / a;

        dt = T / N;
        t = 0:dt:T;

        rhos = rho0 * exp(a*t) + drhot * (exp(a*t) - 1) / a;

    case "slow"
        a = (drho0 - drhot) / rho0^2;
        k = sqrt(a*drhot);
        T = (atan(a*rhoT/k) - atan(a*rho0/k)) / k;

        dt = T / N;
        t = 0:dt:T;

        rhos = tan(k*t+atan(a*rho0/k)) * k / a;

    case "fast"
        rhoStar = rho0;
        a = ((rho0 + rhoStar) * (rhoT + rhoStar) * (drho0 - drhot)) / (rhoT - rho0);
        b = drho0 - a / (rho0 + rhoStar);

        T = (rhoT - rho0) / b ...
            -a / b^2 * log((a + b * (rhoT + rho0))/(a + 2 * b * rho0));

        dt = T / N;
        t = 0:dt:T;

        F = @(rho, tt) (rho - rho0) / b ...
            -(a / b^2) * log((a + b * (rho + rho0))/(a + 2 * b * rho0)) ...
            -tt;

        rhos = zeros(size(t));
        rhos(1) = rho0;

        for i = 2:length(t)
            rhos(i) = fzero(@(rho) F(rho, t(i)), rhos(i-1));
        end
end

r_seq = gen_waypoints(r0, rf, rhos, line);

dt = T / N;

r_cur = r0;
v_cur = v0;

DV = zeros(3, N+1);
traj_r = r_cur;

for k = 1:N


    [Phi_rr, Phi_rv, Phi_vr, Phi_vv] = cw_stm(n, dt);

    r_target = r_seq(:, k+1);

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

DV(:, N+1) = vf - v_cur;

DV_sum = sum(vecnorm(DV));

end

%%
function [Phi_rr, Phi_rv, Phi_vr, Phi_vv] = cw_stm(n, t)

nt = n * t;
s = sin(nt);
c = cos(nt);

Phi_rr = [4 - 3 * c, 0, 0; ...
    6 * (s - nt), 1, 0; ...
    0, 0, c];

Phi_rv = [s / n, 2 * (1 - c) / n, 0; ...
    2 * (c - 1) / n, (4 * s - 3 * nt) / n, 0; ...
    0, 0, s / n];

Phi_vr = [3 * n * s, 0, 0; ...
    6 * n * (c - 1), 0, 0; ...
    0, 0, -n * s];

Phi_vv = [c, 2 * s, 0; ...
    -2 * s, 4 * c - 3, 0; ...
    0, 0, c];

end
%%
function r_seq = gen_waypoints(r0, rf, rhos, type)

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
                -(4 * a^2 * xt^3 + 2 * xt), ...
                (a^2 * xt^4 + xt^2 + rf(3)^2 - rho^2); ...
                ];
            roots_all = roots(coeff);
            roots_real = roots_all(abs(imag(roots_all)) < 1e-8);
            roots_real = real(roots_real);

            if isempty(roots_real)
                error("No valid root found");
            end

            [~, idx] = min(abs(roots_real-r_seq(1, k-1)));
            xm = roots_real(idx);
            ym = a * (xm - xt)^2 + rf(2);

            r_seq(:, k) = [xm; ym; 0];
        end
end

end
