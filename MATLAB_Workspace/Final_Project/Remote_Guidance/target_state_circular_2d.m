function [r_t, v_t, info] = target_state_circular_2d(t, mu, target)
% target_state_circular_2d 计算目标在二维圆轨道上的惯性系状态。

[r_t, v_t, info] = fpkinematics.targetStateCircular2D(t, mu, target);

end
