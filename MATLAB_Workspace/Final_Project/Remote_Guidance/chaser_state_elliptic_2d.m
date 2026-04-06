function [r_c, v_c, info] = chaser_state_elliptic_2d(t, mu, chaser)
% chaser_state_elliptic_2d 计算跟踪器在二维椭圆轨道上的惯性系状态。

M = chaser.M0 + chaser.n * (t - chaser.t0);
E = solve_kepler_ellipse(M, chaser.e);

theta = 2 * atan2( ...
    sqrt(1 + chaser.e) * sin(E / 2), ...
    sqrt(1 - chaser.e) * cos(E / 2));
radius = chaser.a * (1 - chaser.e * cos(E));

r_c = radius * [cos(theta); sin(theta)];
v_c = sqrt(mu / chaser.p) * [-sin(theta); chaser.e + cos(theta)];

info.M = M;
info.E = E;
info.theta = theta;
info.radius = radius;

end
