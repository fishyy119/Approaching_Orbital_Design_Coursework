clear; clc;

%% 常数
mu = 3.986e14;         % 地球引力常数
Re = 6371e3;           
h = 1000e3;
r = Re + h;

n = sqrt(mu / r^3);

%% 初始/终端条件
r0 = [-5000; -500; 0];
v0 = [10; 10; 0];

rf = [-500; -50; 0];
vf = [1; 1; 0];

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

figure; hold on; grid on;
colors = ['r','g','b'];

for k = 1:length(results)
    T = results(k).T;
    traj = generate_trajectory(n, r0, results(k).v0_plus, T, 200);

    plot(traj(1,:), traj(2,:), colors(k), 'LineWidth', 1.5);
end

xlabel('x (m)');
ylabel('y (m)');
legend('T=1000s','T=2000s','T=3000s');
title('CW Transfer Trajectories');
axis equal;

%%
function [Phi_rr, Phi_rv, Phi_vr, Phi_vv] = cw_stm(n, t)

nt = n * t;
s = sin(nt);
c = cos(nt);

Phi_rr = [4-3*c, 0, 0;
          6*(s-nt), 1, 0;
          0, 0, c];

Phi_rv = [s/n, 2*(1-c)/n, 0;
          2*(c-1)/n, (4*s-3*nt)/n, 0;
          0, 0, s/n];

Phi_vr = [3*n*s, 0, 0;
          6*n*(c-1), 0, 0;
          0, 0, -n*s];

Phi_vv = [c, 2*s, 0;
         -2*s, 4*c-3, 0;
          0, 0, c];

end

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