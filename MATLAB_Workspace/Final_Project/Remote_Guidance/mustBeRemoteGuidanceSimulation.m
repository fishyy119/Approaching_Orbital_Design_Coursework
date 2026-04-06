function mustBeRemoteGuidanceSimulation(simulation)
% mustBeRemoteGuidanceSimulation 校验远程导引数值仿真结果结构体。
%
% 要求结果包含 wait、transfer、mission、reference、events、impulses 和 validation
% 等块，且时间向量、二维轨迹矩阵与关键事件量尺寸一致。

if ~isstruct(simulation) || ~isscalar(simulation)
    error('mustBeRemoteGuidanceSimulation:InvalidType', ...
        'simulation 必须为标量 struct。');
end

require_fields(simulation, ["mu", "Re", "best", "wait", "transfer", ...
    "mission", "reference", "events", "impulses", "validation", ...
    "axis_limit_km"], "simulation");

validateattributes(simulation.mu, {'double'}, ...
    {'real', 'finite', 'scalar', 'positive'}, ...
    mfilename, 'simulation.mu');
validateattributes(simulation.Re, {'double'}, ...
    {'real', 'finite', 'scalar', 'positive'}, ...
    mfilename, 'simulation.Re');
validateattributes(simulation.axis_limit_km, {'double'}, ...
    {'real', 'finite', 'scalar', 'positive'}, ...
    mfilename, 'simulation.axis_limit_km');

validate_best(simulation.best);
validate_wait_block(simulation.wait);
validate_transfer_block(simulation.transfer);
validate_mission_block(simulation.mission);
validate_reference_block(simulation.reference);
validate_events(simulation.events);
validate_impulses(simulation.impulses);
validate_validation_block(simulation.validation);

end

function validate_best(best)
% validate_best 校验最优窗口信息。

if ~isstruct(best) || ~isscalar(best)
    error('mustBeRemoteGuidanceSimulation:InvalidBest', ...
        'simulation.best 必须为标量 struct。');
end

require_fields(best, ["td"], "simulation.best");
validateattributes(best.td, {'double'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'}, ...
    mfilename, 'simulation.best.td');

end

function validate_wait_block(wait_block)
% validate_wait_block 校验等待段轨迹块。

if ~isstruct(wait_block) || ~isscalar(wait_block)
    error('mustBeRemoteGuidanceSimulation:InvalidWait', ...
        'simulation.wait 必须为标量 struct。');
end

require_fields(wait_block, ["time", "chaser_pos", "chaser_vel", ...
    "target_pos", "target_vel", "capture_pos", "capture_vel"], ...
    "simulation.wait");

time_count = validate_time_vector(wait_block.time, 'simulation.wait.time');
validate_matrix_2xn(wait_block.chaser_pos, time_count, 'simulation.wait.chaser_pos');
validate_matrix_2xn(wait_block.chaser_vel, time_count, 'simulation.wait.chaser_vel');
validate_matrix_2xn(wait_block.target_pos, time_count, 'simulation.wait.target_pos');
validate_matrix_2xn(wait_block.target_vel, time_count, 'simulation.wait.target_vel');
validate_matrix_2xn(wait_block.capture_pos, time_count, 'simulation.wait.capture_pos');
validate_matrix_2xn(wait_block.capture_vel, time_count, 'simulation.wait.capture_vel');

end

function validate_transfer_block(transfer_block)
% validate_transfer_block 校验转移段轨迹块。

if ~isstruct(transfer_block) || ~isscalar(transfer_block)
    error('mustBeRemoteGuidanceSimulation:InvalidTransfer', ...
        'simulation.transfer 必须为标量 struct。');
end

require_fields(transfer_block, ["time", "time_abs", "chaser_pos", "chaser_vel", ...
    "target_pos", "target_vel", "capture_pos", "capture_vel"], ...
    "simulation.transfer");

time_count = validate_time_vector(transfer_block.time, 'simulation.transfer.time');
validate_time_vector(transfer_block.time_abs, 'simulation.transfer.time_abs', time_count);
validate_matrix_2xn(transfer_block.chaser_pos, time_count, 'simulation.transfer.chaser_pos');
validate_matrix_2xn(transfer_block.chaser_vel, time_count, 'simulation.transfer.chaser_vel');
validate_matrix_2xn(transfer_block.target_pos, time_count, 'simulation.transfer.target_pos');
validate_matrix_2xn(transfer_block.target_vel, time_count, 'simulation.transfer.target_vel');
validate_matrix_2xn(transfer_block.capture_pos, time_count, 'simulation.transfer.capture_pos');
validate_matrix_2xn(transfer_block.capture_vel, time_count, 'simulation.transfer.capture_vel');

end

function validate_mission_block(mission_block)
% validate_mission_block 校验整段任务轨迹块。

if ~isstruct(mission_block) || ~isscalar(mission_block)
    error('mustBeRemoteGuidanceSimulation:InvalidMission', ...
        'simulation.mission 必须为标量 struct。');
end

require_fields(mission_block, ["time", "chaser_pos", "target_pos", "capture_pos"], ...
    "simulation.mission");

time_count = validate_time_vector(mission_block.time, 'simulation.mission.time');
validate_matrix_2xn(mission_block.chaser_pos, time_count, 'simulation.mission.chaser_pos');
validate_matrix_2xn(mission_block.target_pos, time_count, 'simulation.mission.target_pos');
validate_matrix_2xn(mission_block.capture_pos, time_count, 'simulation.mission.capture_pos');

end

function validate_reference_block(reference_block)
% validate_reference_block 校验参考轨道块。

if ~isstruct(reference_block) || ~isscalar(reference_block)
    error('mustBeRemoteGuidanceSimulation:InvalidReference', ...
        'simulation.reference 必须为标量 struct。');
end

require_fields(reference_block, ["target_time", "target_pos", "target_vel", ...
    "chaser_time", "chaser_pos", "chaser_vel"], "simulation.reference");

target_count = validate_time_vector(reference_block.target_time, ...
    'simulation.reference.target_time');
chaser_count = validate_time_vector(reference_block.chaser_time, ...
    'simulation.reference.chaser_time');

validate_matrix_2xn(reference_block.target_pos, target_count, ...
    'simulation.reference.target_pos');
validate_matrix_2xn(reference_block.target_vel, target_count, ...
    'simulation.reference.target_vel');
validate_matrix_2xn(reference_block.chaser_pos, chaser_count, ...
    'simulation.reference.chaser_pos');
validate_matrix_2xn(reference_block.chaser_vel, chaser_count, ...
    'simulation.reference.chaser_vel');

end

function validate_events(events)
% validate_events 校验事件点。

if ~isstruct(events) || ~isscalar(events)
    error('mustBeRemoteGuidanceSimulation:InvalidEvents', ...
        'simulation.events 必须为标量 struct。');
end

require_fields(events, ["start", "departure", "arrival_capture", "arrival_target"], ...
    "simulation.events");

validate_vector_2x1(events.start, 'simulation.events.start');
validate_vector_2x1(events.departure, 'simulation.events.departure');
validate_vector_2x1(events.arrival_capture, 'simulation.events.arrival_capture');
validate_vector_2x1(events.arrival_target, 'simulation.events.arrival_target');

end

function validate_impulses(impulses)
% validate_impulses 校验脉冲量信息。

if ~isstruct(impulses) || ~isscalar(impulses)
    error('mustBeRemoteGuidanceSimulation:InvalidImpulses', ...
        'simulation.impulses 必须为标量 struct。');
end

require_fields(impulses, ["dv1_vec", "dv2_vec_design", "dv2_vec_numeric"], ...
    "simulation.impulses");

validate_vector_2x1(impulses.dv1_vec, 'simulation.impulses.dv1_vec');
validate_vector_2x1(impulses.dv2_vec_design, 'simulation.impulses.dv2_vec_design');
validate_vector_2x1(impulses.dv2_vec_numeric, 'simulation.impulses.dv2_vec_numeric');

end

function validate_validation_block(validation_block)
% validate_validation_block 校验数值校核量。

if ~isstruct(validation_block) || ~isscalar(validation_block)
    error('mustBeRemoteGuidanceSimulation:InvalidValidation', ...
        'simulation.validation 必须为标量 struct。');
end

require_fields(validation_block, ["departure_position_error", ...
    "departure_velocity_error", "arrival_position_error", ...
    "arrival_velocity_error", "capture_velocity_error", ...
    "dv2_numeric_vec", "dv2_numeric", "dv2_vector_error"], ...
    "simulation.validation");

validate_nonnegative_scalar(validation_block.departure_position_error, ...
    'simulation.validation.departure_position_error');
validate_nonnegative_scalar(validation_block.departure_velocity_error, ...
    'simulation.validation.departure_velocity_error');
validate_nonnegative_scalar(validation_block.arrival_position_error, ...
    'simulation.validation.arrival_position_error');
validate_nonnegative_scalar(validation_block.arrival_velocity_error, ...
    'simulation.validation.arrival_velocity_error');
validate_nonnegative_scalar(validation_block.capture_velocity_error, ...
    'simulation.validation.capture_velocity_error');
validate_vector_2x1(validation_block.dv2_numeric_vec, ...
    'simulation.validation.dv2_numeric_vec');
validate_nonnegative_scalar(validation_block.dv2_numeric, ...
    'simulation.validation.dv2_numeric');
validate_nonnegative_scalar(validation_block.dv2_vector_error, ...
    'simulation.validation.dv2_vector_error');

end

function require_fields(struct_value, field_names, struct_name)
% require_fields 校验结构体字段是否齐全。

field_name_list = cellstr(field_names);
missing_fields = field_name_list(~isfield(struct_value, field_name_list));
if ~isempty(missing_fields)
    error('mustBeRemoteGuidanceSimulation:MissingField', ...
        '%s 缺少字段：%s', struct_name, strjoin(missing_fields, ', '));
end

end

function element_count = validate_time_vector(value, value_name, expected_count)
% validate_time_vector 校验时间向量。

if nargin < 3
    expected_count = [];
end

validateattributes(value, {'double'}, {'real', 'finite', 'vector', 'nonempty'}, ...
    mfilename, value_name);

element_count = numel(value);
if ~isempty(expected_count) && element_count ~= expected_count
    error('mustBeRemoteGuidanceSimulation:SizeMismatch', ...
        '%s 的元素个数应为 %d，当前为 %d。', ...
        value_name, expected_count, element_count);
end

end

function validate_matrix_2xn(value, column_count, value_name)
% validate_matrix_2xn 校验二维轨迹矩阵。

validateattributes(value, {'double'}, {'real', 'finite', '2d'}, ...
    mfilename, value_name);

if size(value, 1) ~= 2
    error('mustBeRemoteGuidanceSimulation:SizeMismatch', ...
        '%s 的行数应为 2，当前为 %d。', ...
        value_name, size(value, 1));
end

if size(value, 2) ~= column_count
    error('mustBeRemoteGuidanceSimulation:SizeMismatch', ...
        '%s 的列数应为 %d，当前为 %d。', ...
        value_name, column_count, size(value, 2));
end

end

function validate_vector_2x1(value, value_name)
% validate_vector_2x1 校验二维列向量。

validateattributes(value, {'double'}, {'real', 'finite', 'size', [2, 1]}, ...
    mfilename, value_name);

end

function validate_nonnegative_scalar(value, value_name)
% validate_nonnegative_scalar 校验非负标量。

validateattributes(value, {'double'}, {'real', 'finite', 'scalar', 'nonnegative'}, ...
    mfilename, value_name);

end
