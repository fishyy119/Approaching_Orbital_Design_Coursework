function [r_t, v_t, info] = target_state_circular_2d(t, mu, target)
% target_state_circular_2d 计算目标在二维圆轨道上的惯性系状态。

theta = target.theta0 + target.n * (t - target.t0);
speed = sqrt(mu / target.radius);

r_t = target.radius * [cos(theta); sin(theta)];
v_t = speed * [-sin(theta); cos(theta)];

info.theta = theta;
info.n = target.n;
info.T = target.T;

end
