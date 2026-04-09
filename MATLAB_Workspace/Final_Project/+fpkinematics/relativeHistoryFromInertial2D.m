function [rel_r_history, rel_v_history] = relativeHistoryFromInertial2D( ...
    target_pos, target_vel, chaser_pos, chaser_vel)
% relativeHistoryFromInertial2D 将惯性系状态序列转换为 LVLH 相对状态序列。

arguments
    target_pos (2,:) double
    target_vel (2,:) double
    chaser_pos (2,:) double
    chaser_vel (2,:) double
end

sample_num = size(target_pos, 2);
rel_r_history = zeros(2, sample_num);
rel_v_history = zeros(2, sample_num);

for i = 1:sample_num
    [rel_r_history(:, i), rel_v_history(:, i)] = fpkinematics.inertialToLvlh2D( ...
        target_pos(:, i), target_vel(:, i), chaser_pos(:, i), chaser_vel(:, i));
end

end
