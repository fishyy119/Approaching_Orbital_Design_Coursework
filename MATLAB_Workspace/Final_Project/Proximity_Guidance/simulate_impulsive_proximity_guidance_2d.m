function simulation = simulate_impulsive_proximity_guidance_2d( ...
    mu, Re, target, initial_relative, plan, config)
% simulate_impulsive_proximity_guidance_2d 在惯性系中用 ode113 校核近距离导引设计。
%
% 输入：
%   mu                地球引力参数。
%   Re                地球半径。
%   target            目标圆轨道参数结构体。
%   initial_relative  初始相对状态结构体，包含 rel_r0 与 rel_v0。
%   plan              多脉冲设计结果结构体。
%   config            数值校核配置结构体，包含段内积分步长与切入后
%                     绕飞保持验证时长。
%
% 输出：
%   simulation        数值校核结果结构体，主要包含：
%                     1. segments：各脉冲段的惯性系与 LVLH 系历史；
%                     2. transfer_*：转移段数值与解析轨迹；
%                     3. mission / relative：包含切入后绕飞段的全程轨迹；
%                     4. events / final：各节点事件状态与末端插入状态；
%                     5. post_insert：切入后无控绕飞保持验证历史；
%                     6. validation：解析解与数值积分解的误差统计。
%
% 说明：
%   本函数把每次脉冲视为瞬时速度增量。段内先在惯性系中分别推进目标
%   与跟踪器的两体运动，再逐点回到 LVLH 系，与同一段对应的 CW
%   解析解进行对比；末端切入脉冲施加后，再继续做一段无控传播，
%   检查是否保持在目标绕飞轨道附近。

arguments
    mu (1,1) double {mustBePositive}
    Re (1,1) double {mustBePositive}
    target (1,1) struct
    initial_relative (1,1) struct
    plan (1,1) struct
    config (1,1) struct {mustBeProximitySimulationConfig} = struct( ...
        'segment_step', 10, ...
        'post_insert_duration', 0, ...
        'post_insert_step', 10)
end

[target_pos_0, target_vel_0] = target_state_circular_2d(target.t0, target);
[chaser_pos_0, chaser_vel_0] = lvlh_to_inertial_2d( ...
    target_pos_0, target_vel_0, initial_relative.rel_r0, initial_relative.rel_v0);
segments = repmat(struct( ...
    'index', 0, ...
    'time_local', zeros(1, 0), ...
    'time_abs', zeros(1, 0), ...
    'target_pos', zeros(2, 0), ...
    'target_vel', zeros(2, 0), ...
    'chaser_pos', zeros(2, 0), ...
    'chaser_vel', zeros(2, 0), ...
    'rel_r_numeric', zeros(2, 0), ...
    'rel_v_numeric', zeros(2, 0), ...
    'rel_r_analytic', zeros(2, 0), ...
    'rel_v_analytic', zeros(2, 0), ...
    'position_error', zeros(1, 0), ...
    'velocity_error', zeros(1, 0), ...
    'dv_inertial', zeros(2, 1)), 1, plan.N);

mission_time = [];
mission_target_pos = [];
mission_target_vel = [];
mission_chaser_pos = [];
mission_chaser_vel = [];
mission_rel_r_numeric = [];
mission_rel_v_numeric = [];
mission_rel_r_analytic = [];
mission_rel_v_analytic = [];

node_time = plan.time_nodes;
node_target_pos = zeros(2, plan.N + 1);
node_target_vel = zeros(2, plan.N + 1);
node_chaser_pos = zeros(2, plan.N + 1);
node_chaser_vel = zeros(2, plan.N + 1);
node_rel_r_numeric = zeros(2, plan.N + 1);
node_rel_v_numeric = zeros(2, plan.N + 1);
node_rel_v_design = zeros(2, plan.N + 1);

node_target_pos(:, 1) = target_pos_0;
node_target_vel(:, 1) = target_vel_0;
node_chaser_pos(:, 1) = chaser_pos_0;
node_chaser_vel(:, 1) = chaser_vel_0;
node_rel_r_numeric(:, 1) = initial_relative.rel_r0;
node_rel_v_numeric(:, 1) = initial_relative.rel_v0;
node_rel_v_design(:, 1) = initial_relative.rel_v0;

current_time = target.t0;
current_target_pos = target_pos_0;
current_target_vel = target_vel_0;
current_chaser_pos = chaser_pos_0;
current_chaser_vel = chaser_vel_0;

for k = 1:plan.N
    frame = lvlh_frame_2d(current_target_pos, current_target_vel);
    dv_inertial = frame.C_L2I * plan.dv_segments(:, k);
    chaser_vel_plus = current_chaser_vel + dv_inertial;

    local_time = build_time_grid(plan.dt_segment, config.segment_step);
    abs_time = current_time + local_time;

    [target_pos_hist, target_vel_hist] = propagate_two_body_ode113_2d( ...
        current_target_pos, current_target_vel, local_time, mu);
    [chaser_pos_hist, chaser_vel_hist] = propagate_two_body_ode113_2d( ...
        current_chaser_pos, chaser_vel_plus, local_time, mu);

    [rel_r_numeric_hist, rel_v_numeric_hist] = relative_history_from_inertial_2d( ...
        target_pos_hist, target_vel_hist, chaser_pos_hist, chaser_vel_hist);
    [rel_r_analytic_hist, rel_v_analytic_hist] = cw_history_2d( ...
        target.n, plan.segments(k).rel_r_start, plan.segments(k).rel_v_plus_start, local_time);

    position_error_hist = vecnorm(rel_r_numeric_hist - rel_r_analytic_hist, 2, 1);
    velocity_error_hist = vecnorm(rel_v_numeric_hist - rel_v_analytic_hist, 2, 1);

    segments(k).index = k;
    segments(k).time_local = local_time;
    segments(k).time_abs = abs_time;
    segments(k).target_pos = target_pos_hist;
    segments(k).target_vel = target_vel_hist;
    segments(k).chaser_pos = chaser_pos_hist;
    segments(k).chaser_vel = chaser_vel_hist;
    segments(k).rel_r_numeric = rel_r_numeric_hist;
    segments(k).rel_v_numeric = rel_v_numeric_hist;
    segments(k).rel_r_analytic = rel_r_analytic_hist;
    segments(k).rel_v_analytic = rel_v_analytic_hist;
    segments(k).position_error = position_error_hist;
    segments(k).velocity_error = velocity_error_hist;
    segments(k).dv_inertial = dv_inertial;

    if k == 1
        idx_use = 1:numel(local_time);
    else
        idx_use = 2:numel(local_time);
    end

    mission_time = [mission_time, abs_time(idx_use)]; %#ok<AGROW>
    mission_target_pos = [mission_target_pos, target_pos_hist(:, idx_use)]; %#ok<AGROW>
    mission_target_vel = [mission_target_vel, target_vel_hist(:, idx_use)]; %#ok<AGROW>
    mission_chaser_pos = [mission_chaser_pos, chaser_pos_hist(:, idx_use)]; %#ok<AGROW>
    mission_chaser_vel = [mission_chaser_vel, chaser_vel_hist(:, idx_use)]; %#ok<AGROW>
    mission_rel_r_numeric = [mission_rel_r_numeric, rel_r_numeric_hist(:, idx_use)]; %#ok<AGROW>
    mission_rel_v_numeric = [mission_rel_v_numeric, rel_v_numeric_hist(:, idx_use)]; %#ok<AGROW>
    mission_rel_r_analytic = [mission_rel_r_analytic, rel_r_analytic_hist(:, idx_use)]; %#ok<AGROW>
    mission_rel_v_analytic = [mission_rel_v_analytic, rel_v_analytic_hist(:, idx_use)]; %#ok<AGROW>

    current_time = abs_time(end);
    current_target_pos = target_pos_hist(:, end);
    current_target_vel = target_vel_hist(:, end);
    current_chaser_pos = chaser_pos_hist(:, end);
    current_chaser_vel = chaser_vel_hist(:, end);

    node_target_pos(:, k + 1) = current_target_pos;
    node_target_vel(:, k + 1) = current_target_vel;
    node_chaser_pos(:, k + 1) = current_chaser_pos;
    node_chaser_vel(:, k + 1) = current_chaser_vel;
    node_rel_r_numeric(:, k + 1) = rel_r_numeric_hist(:, end);
    node_rel_v_numeric(:, k + 1) = rel_v_numeric_hist(:, end);
    node_rel_v_design(:, k + 1) = plan.segments(k).rel_v_end_minus;
end

frame_final = lvlh_frame_2d(current_target_pos, current_target_vel);
dv_terminal_inertial = frame_final.C_L2I * plan.dv_terminal;
chaser_vel_inserted = current_chaser_vel + dv_terminal_inertial;

[rel_r_arrival_numeric, rel_v_arrival_numeric] = inertial_to_lvlh_2d( ...
    current_target_pos, current_target_vel, current_chaser_pos, current_chaser_vel);
[rel_r_inserted_numeric, rel_v_inserted_numeric] = inertial_to_lvlh_2d( ...
    current_target_pos, current_target_vel, current_chaser_pos, chaser_vel_inserted);

post_insert_time = build_time_grid(config.post_insert_duration, config.post_insert_step);
post_insert_time_abs = current_time + post_insert_time;
[post_target_pos, post_target_vel] = propagate_two_body_ode113_2d( ...
    current_target_pos, current_target_vel, post_insert_time, mu);
[post_chaser_pos, post_chaser_vel] = propagate_two_body_ode113_2d( ...
    current_chaser_pos, chaser_vel_inserted, post_insert_time, mu);
[post_rel_r_numeric, post_rel_v_numeric] = relative_history_from_inertial_2d( ...
    post_target_pos, post_target_vel, post_chaser_pos, post_chaser_vel);
post_insert_theta = plan.theta + target.n * post_insert_time;
[post_rel_r_reference, post_rel_v_reference] = proximity_reference_orbit_state_2d( ...
    target.n, plan.orbit, post_insert_theta);
post_position_error = vecnorm(post_rel_r_numeric - post_rel_r_reference, 2, 1);
post_velocity_error = vecnorm(post_rel_v_numeric - post_rel_v_reference, 2, 1);

transfer_position_error = vecnorm(mission_rel_r_numeric - mission_rel_r_analytic, 2, 1);
transfer_velocity_error = vecnorm(mission_rel_v_numeric - mission_rel_v_analytic, 2, 1);

mission_time_full = [mission_time, post_insert_time_abs];
mission_target_pos_full = [mission_target_pos, post_target_pos];
mission_target_vel_full = [mission_target_vel, post_target_vel];
mission_chaser_pos_full = [mission_chaser_pos, post_chaser_pos];
mission_chaser_vel_full = [mission_chaser_vel, post_chaser_vel];
mission_rel_r_numeric_full = [mission_rel_r_numeric, post_rel_r_numeric];
mission_rel_v_numeric_full = [mission_rel_v_numeric, post_rel_v_numeric];
mission_rel_r_reference_full = [mission_rel_r_analytic, post_rel_r_reference];
mission_rel_v_reference_full = [mission_rel_v_analytic, post_rel_v_reference];
position_error_all = [transfer_position_error, post_position_error];
velocity_error_all = [transfer_velocity_error, post_velocity_error];

validation = struct();
validation.position_error_history = position_error_all;
validation.velocity_error_history = velocity_error_all;
validation.max_position_error = max(position_error_all);
validation.max_velocity_error = max(velocity_error_all);
validation.rms_position_error = sqrt(mean(position_error_all .^ 2));
validation.rms_velocity_error = sqrt(mean(velocity_error_all .^ 2));
validation.transfer_position_error_history = transfer_position_error;
validation.transfer_velocity_error_history = transfer_velocity_error;
validation.transfer_max_position_error = max(transfer_position_error);
validation.transfer_max_velocity_error = max(transfer_velocity_error);
validation.transfer_rms_position_error = sqrt(mean(transfer_position_error .^ 2));
validation.transfer_rms_velocity_error = sqrt(mean(transfer_velocity_error .^ 2));
validation.arrival_position_error = norm(rel_r_arrival_numeric - plan.rel_rf);
validation.arrival_velocity_error = norm(rel_v_arrival_numeric - plan.rel_vf_minus);
validation.inserted_position_error = norm(rel_r_inserted_numeric - plan.rel_rf);
validation.inserted_velocity_error = norm(rel_v_inserted_numeric - plan.rel_vf);
validation.node_position_error = vecnorm(node_rel_r_numeric - plan.rel_r_seq, 2, 1);
validation.node_velocity_error = vecnorm(node_rel_v_numeric - node_rel_v_design, 2, 1);
validation.post_insert_position_error_history = post_position_error;
validation.post_insert_velocity_error_history = post_velocity_error;
validation.post_insert_max_position_error = max(post_position_error);
validation.post_insert_max_velocity_error = max(post_velocity_error);
validation.post_insert_rms_position_error = sqrt(mean(post_position_error .^ 2));
validation.post_insert_rms_velocity_error = sqrt(mean(post_velocity_error .^ 2));
validation.post_insert_final_position_error = post_position_error(end);
validation.post_insert_final_velocity_error = post_velocity_error(end);

simulation = struct();
simulation.mu = mu;
simulation.Re = Re;
simulation.target = target;
simulation.initial_relative = initial_relative;
simulation.plan = plan;
simulation.segments = segments;
simulation.mission = struct( ...
    'time', mission_time_full, ...
    'target_pos', mission_target_pos_full, ...
    'target_vel', mission_target_vel_full, ...
    'chaser_pos', mission_chaser_pos_full, ...
    'chaser_vel', mission_chaser_vel_full);
simulation.transfer_mission = struct( ...
    'time', mission_time, ...
    'target_pos', mission_target_pos, ...
    'target_vel', mission_target_vel, ...
    'chaser_pos', mission_chaser_pos, ...
    'chaser_vel', mission_chaser_vel);
simulation.relative = struct( ...
    'time', mission_time_full, ...
    'numeric_r', mission_rel_r_numeric_full, ...
    'numeric_v', mission_rel_v_numeric_full, ...
    'analytic_r', mission_rel_r_reference_full, ...
    'analytic_v', mission_rel_v_reference_full);
simulation.transfer_relative = struct( ...
    'time', mission_time, ...
    'numeric_r', mission_rel_r_numeric, ...
    'numeric_v', mission_rel_v_numeric, ...
    'analytic_r', mission_rel_r_analytic, ...
    'analytic_v', mission_rel_v_analytic);
simulation.post_insert = struct( ...
    'time', post_insert_time, ...
    'time_abs', post_insert_time_abs, ...
    'theta', post_insert_theta, ...
    'target_pos', post_target_pos, ...
    'target_vel', post_target_vel, ...
    'chaser_pos', post_chaser_pos, ...
    'chaser_vel', post_chaser_vel, ...
    'rel_r_numeric', post_rel_r_numeric, ...
    'rel_v_numeric', post_rel_v_numeric, ...
    'rel_r_reference', post_rel_r_reference, ...
    'rel_v_reference', post_rel_v_reference, ...
    'position_error', post_position_error, ...
    'velocity_error', post_velocity_error);
simulation.events = struct( ...
    'node_time', node_time, ...
    'target_pos', node_target_pos, ...
    'target_vel', node_target_vel, ...
    'chaser_pos', node_chaser_pos, ...
    'chaser_vel', node_chaser_vel, ...
    'inserted_chaser_vel', chaser_vel_inserted);
simulation.final = struct( ...
    'target_pos', current_target_pos, ...
    'target_vel', current_target_vel, ...
    'rel_r_arrival_numeric', rel_r_arrival_numeric, ...
    'rel_v_arrival_numeric', rel_v_arrival_numeric, ...
    'rel_r_inserted_numeric', rel_r_inserted_numeric, ...
    'rel_v_inserted_numeric', rel_v_inserted_numeric, ...
    'dv_terminal_inertial', dv_terminal_inertial);
simulation.validation = validation;

end

function mustBeProximitySimulationConfig(config)
% mustBeProximitySimulationConfig 校验数值仿真配置。

if ~isstruct(config) || ~isscalar(config)
    error('mustBeProximitySimulationConfig:InvalidType', ...
        'config 必须为标量 struct。');
end

required_fields = {'segment_step', 'post_insert_duration', 'post_insert_step'};
missing_fields = required_fields(~isfield(config, required_fields));
if ~isempty(missing_fields)
    error('mustBeProximitySimulationConfig:MissingField', ...
        'config 缺少字段：%s', strjoin(missing_fields, ', '));
end

validateattributes(config.segment_step, {'double'}, ...
    {'real', 'finite', 'scalar', 'positive'}, ...
    mfilename, 'config.segment_step');
validateattributes(config.post_insert_duration, {'double'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'}, ...
    mfilename, 'config.post_insert_duration');
validateattributes(config.post_insert_step, {'double'}, ...
    {'real', 'finite', 'scalar', 'positive'}, ...
    mfilename, 'config.post_insert_step');

end

function time_grid = build_time_grid(duration, step_size)
% build_time_grid 构造包含起止端点的时间网格。

if duration <= 0
    time_grid = 0;
    return;
end

sample_num = max(2, ceil(duration / step_size) + 1);
time_grid = linspace(0, duration, sample_num);

end

function [position_history, velocity_history] = propagate_two_body_ode113_2d( ...
    r0, v0, time_list, mu)
% propagate_two_body_ode113_2d 用 ode113 推进二维两体轨道。

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
% two_body_dynamics_2d 计算二维两体动力学导数。

r = state(1:2);
v = state(3:4);
r_norm = norm(r);

state_dot = zeros(4, 1);
state_dot(1:2) = v;
state_dot(3:4) = -mu / r_norm^3 * r;

end

function [rel_r_history, rel_v_history] = relative_history_from_inertial_2d( ...
    target_pos, target_vel, chaser_pos, chaser_vel)
% relative_history_from_inertial_2d 将惯性系状态序列转换为 LVLH 相对状态序列。

sample_num = size(target_pos, 2);
rel_r_history = zeros(2, sample_num);
rel_v_history = zeros(2, sample_num);

for i = 1:sample_num
    [rel_r_history(:, i), rel_v_history(:, i)] = inertial_to_lvlh_2d( ...
        target_pos(:, i), target_vel(:, i), chaser_pos(:, i), chaser_vel(:, i));
end

end

function [rel_r_history, rel_v_history] = cw_history_2d(n, rel_r_start, rel_v_plus_start, time_list)
% cw_history_2d 用 CW 状态转移矩阵生成单段内的解析相对状态历史。

sample_num = numel(time_list);
rel_r_history = zeros(2, sample_num);
rel_v_history = zeros(2, sample_num);

for i = 1:sample_num
    tau = time_list(i);
    [Phi_rr, Phi_rv, Phi_vr, Phi_vv] = cw_stm_2d(n, tau);

    rel_r_history(:, i) = Phi_rr * rel_r_start + Phi_rv * rel_v_plus_start;
    rel_v_history(:, i) = Phi_vr * rel_r_start + Phi_vv * rel_v_plus_start;
end

end

function [r_t, v_t] = target_state_circular_2d(t, target)
% target_state_circular_2d 计算目标圆轨道在二维惯性系中的状态。

theta = target.theta0 + target.n * (t - target.t0);
r_t = target.radius * [cos(theta); sin(theta)];
v_t = target.radius * target.n * [-sin(theta); cos(theta)];

end

function frame = lvlh_frame_2d(r_t, v_t)
% lvlh_frame_2d 由目标惯性系状态构造二维 LVLH 坐标基与旋转信息。

r_norm = norm(r_t);
h_z = r_t(1) * v_t(2) - r_t(2) * v_t(1);

if r_norm <= 0
    error('目标位置向量退化，无法构造 LVLH 基。');
end
if abs(h_z) <= eps(max([1, r_norm * norm(v_t)]))
    error('目标角动量过小，无法构造稳定的 LVLH 基。');
end

e_x = r_t / r_norm;
e_y = sign(h_z) * [-e_x(2); e_x(1)];

frame = struct();
frame.C_L2I = [e_x, e_y];
frame.C_I2L = frame.C_L2I.';
frame.n = h_z / r_norm^2;
frame.S = [0, -1; 1, 0];

end

function [r_c, v_c, info] = lvlh_to_inertial_2d(r_t, v_t, rel_r_L, rel_v_L)
% lvlh_to_inertial_2d 将二维 LVLH 相对状态转换为惯性系状态。

info = lvlh_frame_2d(r_t, v_t);
r_c = r_t + info.C_L2I * rel_r_L;
v_c = v_t + info.C_L2I * (rel_v_L + info.n * info.S * rel_r_L);

end

function [rel_r_L, rel_v_L, info] = inertial_to_lvlh_2d(r_t, v_t, r_c, v_c)
% inertial_to_lvlh_2d 将二维惯性系状态转换为 LVLH 相对状态。

info = lvlh_frame_2d(r_t, v_t);
rel_r_L = info.C_I2L * (r_c - r_t);
rel_v_L = info.C_I2L * (v_c - v_t) - info.n * info.S * rel_r_L;

end
