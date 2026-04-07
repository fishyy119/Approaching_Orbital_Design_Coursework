function [Phi_rr, Phi_rv, Phi_vr, Phi_vv] = cw_stm_2d(n, t)
% cw_stm_2d 生成二维共面 CW 方程的状态转移矩阵分块。

nt = n * t;
s = sin(nt);
c = cos(nt);

Phi_rr = [4 - 3 * c, 0; ...
    6 * (s - nt), 1];
Phi_rv = [s / n, 2 * (1 - c) / n; ...
    2 * (c - 1) / n, (4 * s - 3 * nt) / n];
Phi_vr = [3 * n * s, 0; ...
    6 * n * (c - 1), 0];
Phi_vv = [c, 2 * s; ...
    -2 * s, 4 * c - 3];

end
