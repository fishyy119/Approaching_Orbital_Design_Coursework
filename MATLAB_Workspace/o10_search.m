clear;
clc;

%% 常数
mu = 3.986e14;
Re = 6371e3;
h = 1000e3;
r = Re + h;

n = sqrt(mu / r^3);

%% 初始 / 终端条件
r0 = [-5000; -500; 0];
v0 = [10; 10; 0];

rf = [-500; -50; 0];
vf = [1; 1; 0];

T = 3600; % 1h

N_list = 1:12;                 % 离散变量
q_list = linspace(0.2, 1.5, 50);  % 连续变量离散化

DV_map = zeros(length(N_list), length(q_list));

for i = 1:length(N_list)
    N = N_list(i);
    
    for j = 1:length(q_list)
        q = q_list(j);
        
        DV_map(i,j) = eval_total_dv(r0, v0, rf, vf, n, T, N, q);
    end
end

figure;
imagesc(q_list, N_list, DV_map);
colorbar;
xlabel('q');
ylabel('N');
title('Total \DeltaV landscape');

%%
function total_dv = eval_total_dv(r0, v0, rf, vf, n, T, N, q)

dt = T / N;

[r_seq, ~] = gen_geometric_waypoints(r0, v0, rf, vf, N, q);

r_cur = r0;
v_cur = v0;

total_dv = 0;

for k = 1:N
    
    [Phi_rr, Phi_rv, Phi_vr, Phi_vv] = cw_stm(n, dt);

    r_target = r_seq(:, k+1);

    dv = Phi_rv \ (r_target - Phi_rr * r_cur - Phi_rv * v_cur);

    v_cur = v_cur + dv;

    r_next = Phi_rr * r_cur + Phi_rv * v_cur;
    v_next = Phi_vr * r_cur + Phi_vv * v_cur;

    r_cur = r_next;
    v_cur = v_next;

    total_dv = total_dv + norm(dv);
end
total_dv = total_dv + norm(vf-v_cur);

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

function [r_seq, v_seq] = gen_geometric_waypoints(r0, v0, rf, vf, N, q)

r_seq = zeros(3, N+1);
v_seq = zeros(3, N+1);

for k = 0:N

    if abs(q-1) < 1e-8
        alpha = k / N; % 等距（退化情况）
    else
        alpha = (q^k - 1) / (q^N - 1); % 等比
    end

    r_seq(:, k+1) = r0 + alpha * (rf - r0);
    v_seq(:, k+1) = v0 + alpha * (vf - v0);
end

end
