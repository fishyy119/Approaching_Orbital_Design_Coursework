function [r_cap, v_cap, info] = captureStateFromTarget2D( ...
    r_t, v_t, rho_cap_L, rhodot_cap_L)
% captureStateFromTarget2D 按瞬时 LVLH 正交基将捕获点状态转换到二维惯性系。

arguments
    r_t (2,1) double
    v_t (2,1) double
    rho_cap_L (2,1) double
    rhodot_cap_L (2,1) double
end

info = fpkinematics.lvlhFrame2D(r_t, v_t);
r_cap = r_t + info.C_L2I * rho_cap_L;
v_cap = v_t + info.C_L2I * (rhodot_cap_L + info.n * info.S * rho_cap_L);

end
