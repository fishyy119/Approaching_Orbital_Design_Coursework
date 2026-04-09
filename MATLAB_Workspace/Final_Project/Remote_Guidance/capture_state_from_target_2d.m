function [r_cap, v_cap, info] = capture_state_from_target_2d(r_t, v_t, rho_cap_L, rhodot_cap_L)
% capture_state_from_target_2d 按瞬时 LVLH 正交基将捕获点状态转换到二维惯性系。

[r_cap, v_cap, info] = fpkinematics.captureStateFromTarget2D( ...
    r_t, v_t, rho_cap_L, rhodot_cap_L);

end
