function [v1, v2, info] = lambert_universal_2d(r1, r2, dt, mu, branch)
% lambert_universal_2d 用通用变量法求解二维 Lambert 两点转移。
%
% 当前实现范围说明：
%   1. 仅求解零圈（0-rev）Lambert。
%   2. branch 仅区分单圈内的几何 short-way / long-way 两支。
%   3. 不枚举多圈 Lambert 的圈数，也不求解同一圈数下的多根结构。
%
% 输入：
%   r1        转移起点位置。
%   r2        转移终点位置。
%   dt        转移时间。
%   mu        中心天体引力参数。
%   branch    转移支路，取 short 或 long。
%
% 输出：
%   v1        转移起点速度。
%   v2        转移终点速度。
%   info      求解收敛与中间参数信息。

v1 = [nan; nan];
v2 = [nan; nan];
info = struct( ...
    'converged', false, ...
    'iterations', 0, ...
    'branch', string(branch), ...
    'dtheta', nan, ...
    'A', nan, ...
    'z', nan, ...
    'y', nan);

r1_norm = norm(r1);
r2_norm = norm(r2);

if r1_norm <= 0 || r2_norm <= 0 || dt <= 0
    return;
end

dtheta = transfer_angle_2d(r1, r2, branch);
cos_dtheta = cos(dtheta);
sin_dtheta = sin(dtheta);

if abs(1 - cos_dtheta) < 1e-12
    return;
end

A = sin_dtheta * sqrt(r1_norm * r2_norm / (1 - cos_dtheta));
if abs(A) < 1e-12
    return;
end

[z, y, iterations, converged] = solve_lambert_z(r1_norm, r2_norm, A, dt, mu);
if ~converged
    return;
end

f = 1 - y / r1_norm;
g = A * sqrt(y / mu);
gdot = 1 - y / r2_norm;

if abs(g) < 1e-12
    return;
end

v1 = (r2 - f * r1) / g;
v2 = (gdot * r2 - r1) / g;

info.converged = true;
info.iterations = iterations;
info.branch = string(branch);
info.dtheta = dtheta;
info.A = A;
info.z = z;
info.y = y;

end

function dtheta = transfer_angle_2d(r1, r2, branch)
% transfer_angle_2d 计算二维零圈 Lambert 的 short-way / long-way 转移角。

theta1 = atan2(r1(2), r1(1));
theta2 = atan2(r2(2), r2(1));
phi_ccw = mod(theta2 - theta1, 2 * pi);

phi_short = min(phi_ccw, 2 * pi - phi_ccw);
phi_long = 2 * pi - phi_short;

if strcmpi(branch, 'short')
    dtheta = phi_short;
else
    dtheta = phi_long;
end

end

function [z, y, iterations, converged] = solve_lambert_z(r1_norm, r2_norm, A, dt, mu)
% solve_lambert_z 用括界加二分法求解零圈 Lambert 的通用变量 z。

z = nan;
y = nan;
iterations = 0;
converged = false;

tol_F = 1e-8;
tol_z = 1e-10;
z_pos_limit = 4 * pi^2 - 1e-6; % 零圈椭圆分支的正向搜索上界
z_neg_limit = -256;

[z_ref, F_ref, y_ref, iterations, valid_ref] = find_valid_reference_z( ...
    r1_norm, r2_norm, A, dt, mu, z_pos_limit);
if ~valid_ref
    return;
end

if abs(F_ref) < tol_F
    z = z_ref;
    y = y_ref;
    converged = true;
    return;
end

step_init = 0.25;

if F_ref < 0
    [z_low, z_high, iterations, bracketed, converged, z, y] = ...
        expand_bracket_from_reference( ...
            z_ref, step_init, 1, z_pos_limit, 40, tol_F, ...
            r1_norm, r2_norm, A, dt, mu, iterations);
else
    [z_low, z_high, iterations, bracketed, converged, z, y] = ...
        expand_bracket_from_reference( ...
            z_ref, step_init, -1, z_neg_limit, 50, tol_F, ...
            r1_norm, r2_norm, A, dt, mu, iterations);
end

if converged || ~bracketed
    return;
end

for k = 1:80
    z_mid = 0.5 * (z_low + z_high);
    [F_mid, y_mid, valid_mid, iterations] = evaluate_lambert_residual( ...
        z_mid, r1_norm, r2_norm, A, dt, mu, iterations);

    if ~valid_mid
        return;
    end

    z = z_mid;
    y = y_mid;

    if abs(F_mid) < tol_F || abs(z_high - z_low) < tol_z
        converged = true;
        return;
    end

    if F_mid > 0
        z_high = z_mid;
    else
        z_low = z_mid;
    end
end

end

function [z_low, z_high, iterations, bracketed, converged, z_root, y_root] = ...
    expand_bracket_from_reference( ...
        z_ref, step_init, direction, z_limit, max_iter, tol_F, ...
        r1_norm, r2_norm, A, dt, mu, iterations)
% expand_bracket_from_reference 从参考点单侧扩界，直到获得可二分的括界。
%
% 约定：
%   1. direction = 1 时，从 F(z_ref) < 0 的参考点向正方向搜索，目标是找到 F >= 0。
%   2. direction = -1 时，从 F(z_ref) > 0 的参考点向负方向搜索，目标是找到 F <= 0。

z_low = nan;
z_high = nan;
bracketed = false;
converged = false;
z_root = nan;
y_root = nan;
step = step_init;

if direction > 0
    z_low = z_ref;
else
    z_high = z_ref;
end

for k = 1:max_iter
    if direction > 0
        z_trial = min(z_low + step, z_limit);
    else
        z_trial = max(z_high - step, z_limit);
    end

    [F_trial, y_trial, valid_trial, iterations] = evaluate_lambert_residual( ...
        z_trial, r1_norm, r2_norm, A, dt, mu, iterations);

    if valid_trial
        if abs(F_trial) < tol_F
            z_root = z_trial;
            y_root = y_trial;
            converged = true;
            return;
        end

        if direction > 0
            if F_trial >= 0
                z_high = z_trial;
                bracketed = true;
                return;
            end

            z_low = z_trial;
        else
            if F_trial <= 0
                z_low = z_trial;
                bracketed = true;
                return;
            end

            z_high = z_trial;
        end
    end

    if (direction > 0 && z_trial >= z_limit) || ...
            (direction < 0 && z_trial <= z_limit)
        return;
    end

    step = 2 * step;
end

end

function [z_ref, F_ref, y_ref, iterations, valid_ref] = find_valid_reference_z( ...
    r1_norm, r2_norm, A, dt, mu, z_pos_limit)
% find_valid_reference_z 从 z = 0 起搜索第一个可用参考点。

z_ref = 0;
F_ref = nan;
y_ref = nan;
iterations = 0;
valid_ref = false;
step = 0.25;

for k = 1:20
    [F_ref, y_ref, valid_ref, iterations] = evaluate_lambert_residual( ...
        z_ref, r1_norm, r2_norm, A, dt, mu, iterations);

    if valid_ref
        return;
    end

    z_ref = z_ref + step;
    if z_ref >= z_pos_limit
        return;
    end

    step = 2 * step;
end

end

function [F, y, valid, iterations] = evaluate_lambert_residual( ...
    z, r1_norm, r2_norm, A, dt, mu, iterations)
% evaluate_lambert_residual 计算残差并累计一次函数评估计数。

[F, y, valid] = lambert_residual(z, r1_norm, r2_norm, A, dt, mu);
iterations = iterations + 1;

end

function [F, y, valid] = lambert_residual(z, r1_norm, r2_norm, A, dt, mu)
% lambert_residual 计算零圈 Lambert 时间方程残差。

[C, S] = stumpff_cs(z);
valid = isfinite(C) && isfinite(S) && C > 0;

if ~valid
    F = nan;
    y = nan;
    return;
end

y = r1_norm + r2_norm + A * (z * S - 1) / sqrt(C);
if y <= 0
    valid = false;
    F = nan;
    return;
end

chi = sqrt(y / C);
sqrt_y = sqrt(y);
F = chi^3 * S + A * sqrt_y - sqrt(mu) * dt;
valid = isfinite(F);

end

function [C, S] = stumpff_cs(z)
% stumpff_cs 计算通用变量法中的 Stumpff 函数。

if z > 1e-8
    root_z = sqrt(z);
    S = (root_z - sin(root_z)) / root_z^3;
    C = (1 - cos(root_z)) / z;
elseif z < -1e-8
    root_minus_z = sqrt(-z);
    S = (sinh(root_minus_z) - root_minus_z) / root_minus_z^3;
    C = (cosh(root_minus_z) - 1) / (-z);
else
    S = 1 / 6;
    C = 1 / 2;
end

end
