function simulation = simulate_impulsive_remote_guidance_2d( ...
    mu, Re, target, chaser, best, rho_cap_L, rhodot_cap_L, config)
% simulate_impulsive_remote_guidance_2d 基于机动事件与 ode113 积分对远程导引过程做二维数值递推。
%
% 输入：
%   mu            地球引力参数。
%   Re            地球半径。
%   target        目标轨道参数结构体。
%   chaser        跟踪器轨道参数结构体。
%   best          最优 Lambert 窗口结果结构体。
%   rho_cap_L     捕获点在 LVLH 系中的位置。
%   rhodot_cap_L  捕获点在 LVLH 系中的速度。
%   config        数值递推配置结构体。
%
% 输出：
%   simulation    用于绘图、动画和校核的仿真结果结构体。

arguments
    mu (1,1) double {mustBePositive}
    Re (1,1) double {mustBePositive}
    target (1,1) struct
    chaser (1,1) struct
    best (1,1) struct
    rho_cap_L (2,1) double
    rhodot_cap_L (2,1) double
    config (1,1) struct {mustBeRemoteGuidanceSimulationConfig} = struct( ...
        'wait_step', 30, ...
        'transfer_step', 20, ...
        'reference_orbit_samples', 720)
end

[r_chaser0, v_chaser0] = chaser_state_elliptic_2d(chaser.t0, mu, chaser);
[r_target0, v_target0] = target_state_circular_2d(target.t0, mu, target);

wait_time = build_time_grid(best.td, config.wait_step);
transfer_time = build_time_grid(best.dt, config.transfer_step);
transfer_time_abs = best.td + transfer_time;

[wait_chaser_pos, wait_chaser_vel] = propagate_two_body_ode113_2d( ...
    r_chaser0, v_chaser0, wait_time, mu);
[wait_target_pos, wait_target_vel] = propagate_two_body_ode113_2d( ...
    r_target0, v_target0, wait_time, mu);
[wait_capture_pos, wait_capture_vel] = capture_history_from_target_2d( ...
    wait_target_pos, wait_target_vel, rho_cap_L, rhodot_cap_L);

departure_pos = wait_chaser_pos(:, end);
departure_vel_minus = wait_chaser_vel(:, end);
departure_vel_plus = departure_vel_minus + best.dv1_vec;

[transfer_chaser_pos, transfer_chaser_vel] = propagate_two_body_ode113_2d( ...
    departure_pos, departure_vel_plus, transfer_time, mu);
[transfer_target_pos, transfer_target_vel] = propagate_two_body_ode113_2d( ...
    wait_target_pos(:, end), wait_target_vel(:, end), transfer_time, mu);
[transfer_capture_pos, transfer_capture_vel] = capture_history_from_target_2d( ...
    transfer_target_pos, transfer_target_vel, rho_cap_L, rhodot_cap_L);

mission_time = [wait_time, transfer_time_abs(2:end)];
mission_chaser_pos = [wait_chaser_pos, transfer_chaser_pos(:, 2:end)];
mission_target_pos = [wait_target_pos, transfer_target_pos(:, 2:end)];
mission_capture_pos = [wait_capture_pos, transfer_capture_pos(:, 2:end)];

reference_time_target = linspace(0, target.T, config.reference_orbit_samples);
reference_time_chaser = linspace(0, chaser.T, config.reference_orbit_samples);
[target_reference_pos, target_reference_vel] = propagate_two_body_ode113_2d( ...
    r_target0, v_target0, reference_time_target, mu);
[chaser_reference_pos, chaser_reference_vel] = propagate_two_body_ode113_2d( ...
    r_chaser0, v_chaser0, reference_time_chaser, mu);

arrival_pos_numeric = transfer_chaser_pos(:, end);
arrival_vel_numeric = transfer_chaser_vel(:, end);
capture_pos_numeric = transfer_capture_pos(:, end);
capture_vel_numeric = transfer_capture_vel(:, end);
dv2_numeric_vec = capture_vel_numeric - arrival_vel_numeric;

validation = struct();
validation.departure_position_error = norm(departure_pos - best.r1);
validation.departure_velocity_error = norm(departure_vel_minus - best.v1_nom);
validation.arrival_position_error = norm(arrival_pos_numeric - capture_pos_numeric);
validation.arrival_velocity_error = norm(arrival_vel_numeric - best.v2_tr);
validation.capture_velocity_error = norm(capture_vel_numeric - best.vf);
validation.dv2_numeric_vec = dv2_numeric_vec;
validation.dv2_numeric = norm(dv2_numeric_vec);
validation.dv2_vector_error = norm(dv2_numeric_vec - best.dv2_vec);

all_radius = [vecnorm(target_reference_pos, 2, 1), ...
    vecnorm(chaser_reference_pos, 2, 1), ...
    vecnorm(mission_chaser_pos, 2, 1), ...
    vecnorm(mission_target_pos, 2, 1), ...
    vecnorm(mission_capture_pos, 2, 1), Re];
axis_limit_km = 1.15 * max(all_radius) / 1e3;

simulation = struct();
simulation.mu = mu;
simulation.Re = Re;
simulation.best = best;
simulation.wait = struct( ...
    'time', wait_time, ...
    'chaser_pos', wait_chaser_pos, ...
    'chaser_vel', wait_chaser_vel, ...
    'target_pos', wait_target_pos, ...
    'target_vel', wait_target_vel, ...
    'capture_pos', wait_capture_pos, ...
    'capture_vel', wait_capture_vel);
simulation.transfer = struct( ...
    'time', transfer_time, ...
    'time_abs', transfer_time_abs, ...
    'chaser_pos', transfer_chaser_pos, ...
    'chaser_vel', transfer_chaser_vel, ...
    'target_pos', transfer_target_pos, ...
    'target_vel', transfer_target_vel, ...
    'capture_pos', transfer_capture_pos, ...
    'capture_vel', transfer_capture_vel);
simulation.mission = struct( ...
    'time', mission_time, ...
    'chaser_pos', mission_chaser_pos, ...
    'target_pos', mission_target_pos, ...
    'capture_pos', mission_capture_pos);
simulation.reference = struct( ...
    'target_time', reference_time_target, ...
    'target_pos', target_reference_pos, ...
    'target_vel', target_reference_vel, ...
    'chaser_time', reference_time_chaser, ...
    'chaser_pos', chaser_reference_pos, ...
    'chaser_vel', chaser_reference_vel);
simulation.events = struct( ...
    'start', wait_chaser_pos(:, 1), ...
    'departure', departure_pos, ...
    'arrival_capture', capture_pos_numeric, ...
    'arrival_target', transfer_target_pos(:, end));
simulation.impulses = struct( ...
    'dv1_vec', best.dv1_vec, ...
    'dv2_vec_design', best.dv2_vec, ...
    'dv2_vec_numeric', dv2_numeric_vec);
simulation.validation = validation;
simulation.axis_limit_km = axis_limit_km;

end

function mustBeRemoteGuidanceSimulationConfig(config)
% mustBeRemoteGuidanceSimulationConfig 校验数值仿真配置结构体。

if ~isstruct(config) || ~isscalar(config)
    error('mustBeRemoteGuidanceSimulationConfig:InvalidType', ...
        'config 必须为标量 struct。');
end

required_fields = {'wait_step', 'transfer_step', 'reference_orbit_samples'};
missing_fields = required_fields(~isfield(config, required_fields));
if ~isempty(missing_fields)
    error('mustBeRemoteGuidanceSimulationConfig:MissingField', ...
        'config 缺少字段：%s', strjoin(missing_fields, ', '));
end

validateattributes(config.wait_step, {'double'}, ...
    {'real', 'finite', 'scalar', 'positive'}, ...
    mfilename, 'config.wait_step');
validateattributes(config.transfer_step, {'double'}, ...
    {'real', 'finite', 'scalar', 'positive'}, ...
    mfilename, 'config.transfer_step');
validateattributes(config.reference_orbit_samples, {'double'}, ...
    {'real', 'finite', 'scalar', 'positive'}, ...
    mfilename, 'config.reference_orbit_samples');
mustBeInteger(config.reference_orbit_samples);

end

function time_grid = build_time_grid(duration, step_size)
% build_time_grid 构造包含起终点的时间网格。

if duration <= 0
    time_grid = 0;
    return;
end

num_samples = max(2, ceil(duration / step_size) + 1);
time_grid = linspace(0, duration, num_samples);

end

function [position_history, velocity_history] = propagate_two_body_ode113_2d( ...
    r0, v0, time_list, mu)
% propagate_two_body_ode113_2d 用 ode113 按给定采样时刻推进二维两体轨迹。

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
    @(t, state) two_body_dynamics_2d(t, state, mu), ...
    time_nodes, initial_state, ode_options);

position_history = state_history(:, 1:2).';
velocity_history = state_history(:, 3:4).';

end

function state_dot = two_body_dynamics_2d(~, state, mu)
% two_body_dynamics_2d 计算二维两体动力学状态导数。

r = state(1:2);
v = state(3:4);
r_norm = norm(r);

state_dot = zeros(4, 1);
state_dot(1:2) = v;
state_dot(3:4) = -mu / r_norm^3 * r;

end

function [capture_pos, capture_vel] = capture_history_from_target_2d( ...
    target_pos, target_vel, rho_cap_L, rhodot_cap_L)
% capture_history_from_target_2d 由目标状态序列生成捕获点状态序列。

capture_pos = zeros(size(target_pos));
capture_vel = zeros(size(target_vel));

for i = 1:size(target_pos, 2)
    [capture_pos(:, i), capture_vel(:, i)] = capture_state_from_target_2d( ...
        target_pos(:, i), target_vel(:, i), rho_cap_L, rhodot_cap_L);
end

end
