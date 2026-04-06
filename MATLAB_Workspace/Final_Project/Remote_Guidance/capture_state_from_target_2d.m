function [r_cap, v_cap, info] = capture_state_from_target_2d(r_t, v_t, rho_cap_L, rhodot_cap_L)
% capture_state_from_target_2d 按瞬时 LVLH 正交基将捕获点状态转换到二维惯性系。

r_norm = norm(r_t);
h_z = r_t(1) * v_t(2) - r_t(2) * v_t(1);

if r_norm <= 0
    error('目标位置向量退化，无法构造 LVLH 基。');
end
if abs(h_z) <= eps(max([1, r_norm * norm(v_t)]))
    error('目标角动量过小，无法构造稳定的 LVLH 基。');
end

e_x = r_t / r_norm;
e_y = sign(h_z) * [-e_x(2); e_x(1)];
C_L2I = [e_x, e_y];
C_I2L = C_L2I.';
n = h_z / r_norm^2;

S = [0, -1; 1, 0];

r_cap = r_t + C_L2I * rho_cap_L;
v_cap = v_t + C_L2I * (rhodot_cap_L + n * S * rho_cap_L);

info.C_L2I = C_L2I;
info.C_I2L = C_I2L;
info.n = n;
info.h_z = h_z;

end
