function [r_t, v_t, info] = targetStateCircular2D(t, mu, target)
% targetStateCircular2D 计算目标在二维圆轨道上的惯性系状态。

arguments
    t (1,1) double {mustBeReal, mustBeFinite}
    mu (1,1) double {mustBePositive}
    target (1,1) struct
end

theta = target.theta0 + target.n * (t - target.t0);
speed = sqrt(mu / target.radius);

r_t = target.radius * [cos(theta); sin(theta)];
v_t = speed * [-sin(theta); cos(theta)];

info = struct();
info.theta = theta;
info.n = target.n;
info.T = target.T;

end
