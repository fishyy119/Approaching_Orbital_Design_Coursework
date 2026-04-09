function [capture_pos, capture_vel] = captureHistoryFromTarget2D( ...
    target_pos, target_vel, rho_cap_L, rhodot_cap_L)
% captureHistoryFromTarget2D 由目标状态序列生成捕获点状态序列。

arguments
    target_pos (2,:) double
    target_vel (2,:) double
    rho_cap_L (2,1) double
    rhodot_cap_L (2,1) double
end

capture_pos = zeros(size(target_pos));
capture_vel = zeros(size(target_vel));

for i = 1:size(target_pos, 2)
    [capture_pos(:, i), capture_vel(:, i)] = fpkinematics.captureStateFromTarget2D( ...
        target_pos(:, i), target_vel(:, i), rho_cap_L, rhodot_cap_L);
end

end
