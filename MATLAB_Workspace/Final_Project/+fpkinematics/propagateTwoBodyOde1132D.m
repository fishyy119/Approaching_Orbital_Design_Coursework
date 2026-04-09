function [position_history, velocity_history] = propagateTwoBodyOde1132D( ...
    r0, v0, time_list, mu)
% propagateTwoBodyOde1132D 用 ode113 按给定采样时刻推进二维两体轨迹。

arguments
    r0 (2,1) double
    v0 (2,1) double
    time_list (1,:) double {mustBeReal, mustBeFinite}
    mu (1,1) double {mustBePositive}
end

position_history = zeros(2, numel(time_list));
velocity_history = zeros(2, numel(time_list));

if isempty(time_list)
    return;
end

position_history(:, 1) = r0;
velocity_history(:, 1) = v0;

if numel(time_list) == 1
    return;
end

time_nodes = time_list(:);
initial_state = [r0; v0];
ode_options = odeset( ...
    'RelTol', 1e-10, ...
    'AbsTol', [1e-3, 1e-3, 1e-6, 1e-6], ...
    'MaxStep', max(diff(time_nodes)));

[~, state_history] = ode113( ...
    @(t, state) twoBodyDynamics2D(state, mu), ...
    time_nodes, initial_state, ode_options);

position_history = state_history(:, 1:2).';
velocity_history = state_history(:, 3:4).';

end

function state_dot = twoBodyDynamics2D(state, mu)
% twoBodyDynamics2D 计算二维两体动力学状态导数。

r = state(1:2);
v = state(3:4);
r_norm = norm(r);

state_dot = zeros(4, 1);
state_dot(1:2) = v;
state_dot(3:4) = -mu / r_norm^3 * r;

end
