function [r_min, info] = transfer_arc_min_radius_2d(r1, v1, r2, v2, mu)
% transfer_arc_min_radius_2d 解析计算单圈 Lambert 转移弧段的最小半径。
%
% 输入：
%   r1        转移起点位置。
%   v1        转移起点速度。
%   r2        转移终点位置。
%   v2        转移终点速度。
%   mu        中心天体引力参数。
%
% 输出：
%   r_min     实际转移弧段上的最小半径；非有界或异常输入直接返回 0。
%   info      轨道类型、近地点参数与一致性诊断信息。
%
% 说明：
%   本函数仅适用于当前零圈 Lambert 求解器输出的单圈 short-way / long-way 弧段，
%   不考虑多圈 Lambert。

arguments
    r1 (2,1) double
    v1 (2,1) double
    r2 (2,1) double
    v2 (2,1) double
    mu (1,1) double {mustBePositive}
end

tol_h = 1e-12;
tol_e = 1e-10;
tol_consistency = 1e-8;

r1_norm = norm(r1);
r2_norm = norm(r2);
r_min = 0;

info = struct( ...
    'conic_type', "invalid", ...
    'ecc', nan, ...
    'rp', nan, ...
    'u1', nan, ...
    'u2', nan, ...
    'delta_u', nan, ...
    'crosses_periapsis', false, ...
    'energy_error', nan, ...
    'angular_momentum_error', nan);

if r1_norm <= 0 || r2_norm <= 0
    return;
end

h1_z = cross2d(r1, v1);
h2_z = cross2d(r2, v2);
energy1 = 0.5 * dot(v1, v1) - mu / r1_norm;
energy2 = 0.5 * dot(v2, v2) - mu / r2_norm;

info.energy_error = abs(energy2 - energy1);
info.angular_momentum_error = abs(h2_z - h1_z);

if abs(h1_z) <= tol_h
    return;
end

energy_scale = max([1, abs(energy1), abs(energy2)]);
angular_momentum_scale = max([1, abs(h1_z), abs(h2_z)]);

if info.energy_error > tol_consistency * energy_scale || ...
        info.angular_momentum_error > tol_consistency * angular_momentum_scale
    return;
end

p = h1_z^2 / mu;
e_vec = ((dot(v1, v1) - mu / r1_norm) * r1 - dot(r1, v1) * v1) / mu;
ecc = norm(e_vec);
rp = p / (1 + ecc);

info.ecc = ecc;
info.rp = rp;

if energy1 >= 0 || ecc >= 1 - tol_e
    info.conic_type = "unbounded_rejected";
    return;
end

if ecc < tol_e
    info.conic_type = "circular";
    r_min = min([r1_norm, r2_norm, rp]);
    return;
end

motion_sign = sign(h1_z);
e_hat = e_vec / ecc;
q_hat = motion_sign * [-e_hat(2); e_hat(1)];

u1 = wrap_to_2pi(atan2(dot(r1, q_hat), dot(r1, e_hat)));
u2 = wrap_to_2pi(atan2(dot(r2, q_hat), dot(r2, e_hat)));
delta_u = mod(u2 - u1, 2 * pi);
crosses_periapsis = u2 < u1;

info.u1 = u1;
info.u2 = u2;
info.delta_u = delta_u;
info.crosses_periapsis = crosses_periapsis;
info.conic_type = "elliptic";

if crosses_periapsis && isfinite(rp)
    r_min = min([r1_norm, r2_norm, rp]);
else
    r_min = min(r1_norm, r2_norm);
end

end

function value = cross2d(a, b)
% cross2d 计算二维向量叉积的 z 分量。

value = a(1) * b(2) - a(2) * b(1);
end

function angle_wrapped = wrap_to_2pi(angle)
% wrap_to_2pi 将角度封装到 [0, 2pi)。

angle_wrapped = mod(angle, 2 * pi);
end
