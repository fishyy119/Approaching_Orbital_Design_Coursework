function plan = proximity_solve_line_multimpulse_2d( ...
    n, rel_r0, rel_v0, rel_rf, rel_vf, config)
% proximity_solve_line_multimpulse_2d 用直线指数趋近律求解二维多脉冲转移。
%
% 输入：
%   n        目标圆轨道角速度。
%   rel_r0   初始相对位置向量。
%   rel_v0   初始相对速度向量。
%   rel_rf   切入点相对位置向量。
%   rel_vf   切入点相对速度向量。
%   config   多脉冲转移配置结构体，包含指数趋近律调度参数与总转移时间上限。
%
% 输出：
%   plan     转移方案结构体，包含每段脉冲、节点状态、段内解析历史、
%            末端切入脉冲以及总 Δv。
%
% 说明：
%   1. 几何路径固定为起点到切入点之间的直线。
%   2. 标量 rho_nodes 表示沿该直线的剩余距离，不再与相对位置向量混用。
%   3. 每段段首脉冲通过 CW 状态转移矩阵反解，使该段末端精确落在下一节点。
%   4. 若由趋近律反推得到的总转移时间超过上限，则该解直接判为不可行。
%   5. rho(t) 与 T_total 的计算形式与 CW_Transfer/Timpulse.m 的指数型
%      参考实现保持一致；drho0、drhoT 在当前函数中作为节点调度参数使用。

arguments
    n (1,1) double {mustBePositive}
    rel_r0 (2,1) double
    rel_v0 (2,1) double
    rel_rf (2,1) double
    rel_vf (2,1) double
    config (1,1) struct {mustBeProximityTransferConfig}
end

plan = struct();
plan.is_feasible = false;
plan.failure_reason = "";
plan.time_limit_exceeded = false;
plan.N = config.N;
plan.drho0 = config.drho0;
plan.drhoT = config.drhoT;
plan.T_total_max = config.T_total_max;
plan.theta = NaN;

distance0 = norm(rel_r0 - rel_rf);
if distance0 <= 0
    plan.failure_reason = "初始点与切入点重合，无法构造直线路径。";
    return;
end

a = (config.drho0 - config.drhoT) / distance0;
if abs(a) < 1e-12
    plan.failure_reason = "指数趋近律参数退化。";
    return;
end

T_total = log(config.drhoT / config.drho0) / a;
if ~isfinite(T_total) || T_total <= 0
    plan.failure_reason = "由给定趋近参数得到的总转移时间无效。";
    return;
end

if T_total > config.T_total_max
    plan.failure_reason = sprintf( ...
        '总转移时间 %.2f s 超过上限 %.2f s。', ...
        T_total, config.T_total_max);
    plan.time_limit_exceeded = true;
    return;
end

dt_segment = T_total / config.N;
[Phi_rr, Phi_rv, Phi_vr, Phi_vv] = cw_stm_2d(n, dt_segment);

if rcond(Phi_rv) < 1e-10
    plan.failure_reason = "段间 Phi_rv 近似奇异，无法稳定反解脉冲。";
    return;
end

t_nodes = linspace(0, T_total, config.N + 1);
rho_nodes = distance0 * exp(a * t_nodes) ...
    + config.drhoT * (exp(a * t_nodes) - 1) / a;
rhodot_schedule_nodes = config.drho0 * exp(a * t_nodes);
rho_nodes(end) = 0;
rhodot_schedule_nodes(end) = config.drhoT;

path_hat = (rel_r0 - rel_rf) / distance0;
rel_r_seq = rel_rf + path_hat * rho_nodes;
rel_r_seq(:, 1) = rel_r0;
rel_r_seq(:, end) = rel_rf;

segments = repmat(struct( ...
    'index', 0, ...
    't_start', 0, ...
    't_end', 0, ...
    'rel_r_start', zeros(2, 1), ...
    'rel_v_minus_start', zeros(2, 1), ...
    'rel_v_plus_start', zeros(2, 1), ...
    'rel_r_end', zeros(2, 1), ...
    'rel_v_end_minus', zeros(2, 1), ...
    'dv', zeros(2, 1)), 1, config.N);

dv_segments = zeros(2, config.N);
rel_r_cur = rel_r0;
rel_v_minus_cur = rel_v0;

history_time = [];
history_rel_r = [];
history_rel_v = [];

for k = 1:config.N
    rel_r_target = rel_r_seq(:, k + 1);
    dv_k = Phi_rv \ (rel_r_target - Phi_rr * rel_r_cur - Phi_rv * rel_v_minus_cur);
    rel_v_plus_cur = rel_v_minus_cur + dv_k;

    local_time = linspace(0, dt_segment, config.segment_samples);
    rel_r_segment = zeros(2, numel(local_time));
    rel_v_segment = zeros(2, numel(local_time));

    for i = 1:numel(local_time)
        tau = local_time(i);
        [Phi_rr_tau, Phi_rv_tau, Phi_vr_tau, Phi_vv_tau] = cw_stm_2d(n, tau);

        rel_r_segment(:, i) = Phi_rr_tau * rel_r_cur + Phi_rv_tau * rel_v_plus_cur;
        rel_v_segment(:, i) = Phi_vr_tau * rel_r_cur + Phi_vv_tau * rel_v_plus_cur;
    end

    rel_r_next = Phi_rr * rel_r_cur + Phi_rv * rel_v_plus_cur;
    rel_v_next_minus = Phi_vr * rel_r_cur + Phi_vv * rel_v_plus_cur;

    segments(k).index = k;
    segments(k).t_start = t_nodes(k);
    segments(k).t_end = t_nodes(k + 1);
    segments(k).rel_r_start = rel_r_cur;
    segments(k).rel_v_minus_start = rel_v_minus_cur;
    segments(k).rel_v_plus_start = rel_v_plus_cur;
    segments(k).rel_r_end = rel_r_next;
    segments(k).rel_v_end_minus = rel_v_next_minus;
    segments(k).dv = dv_k;

    if k == 1
        index_use = 1:numel(local_time);
    else
        index_use = 2:numel(local_time);
    end

    history_time = [history_time, t_nodes(k) + local_time(index_use)]; %#ok<AGROW>
    history_rel_r = [history_rel_r, rel_r_segment(:, index_use)]; %#ok<AGROW>
    history_rel_v = [history_rel_v, rel_v_segment(:, index_use)]; %#ok<AGROW>

    dv_segments(:, k) = dv_k;
    rel_r_cur = rel_r_next;
    rel_v_minus_cur = rel_v_next_minus;
end

dv_terminal = rel_vf - rel_v_minus_cur;
dv_all = [dv_segments, dv_terminal];

plan.is_feasible = true;
plan.failure_reason = "";
plan.time_limit_exceeded = false;
plan.N = config.N;
plan.drho0 = config.drho0;
plan.drhoT = config.drhoT;
plan.T_total_max = config.T_total_max;
plan.rel_r0 = rel_r0;
plan.rel_v0 = rel_v0;
plan.rel_rf = rel_rf;
plan.rel_vf = rel_vf;
plan.rel_vf_minus = rel_v_minus_cur;
plan.distance0 = distance0;
plan.schedule_a = a;
plan.T_total = T_total;
plan.dt_segment = dt_segment;
plan.time_nodes = t_nodes;
plan.rho_nodes = rho_nodes;
plan.rhodot_schedule_nodes = rhodot_schedule_nodes;
plan.rel_r_seq = rel_r_seq;
plan.dv_segments = dv_segments;
plan.dv_terminal = dv_terminal;
plan.dv_total = sum(vecnorm(dv_all, 2, 1));
plan.segments = segments;
plan.history = struct( ...
    'time', history_time, ...
    'rel_r', history_rel_r, ...
    'rel_v', history_rel_v);

end

function mustBeProximityTransferConfig(config)
% mustBeProximityTransferConfig 校验多脉冲趋近配置。

if ~isstruct(config) || ~isscalar(config)
    error('mustBeProximityTransferConfig:InvalidType', ...
        'config 必须为标量 struct。');
end

required_fields = {'N', 'drho0', 'drhoT', 'segment_samples', 'T_total_max'};
missing_fields = required_fields(~isfield(config, required_fields));
if ~isempty(missing_fields)
    error('mustBeProximityTransferConfig:MissingField', ...
        'config 缺少字段：%s', strjoin(missing_fields, ', '));
end

validateattributes(config.N, {'double'}, ...
    {'real', 'finite', 'scalar', 'positive'}, ...
    mfilename, 'config.N');
mustBeInteger(config.N);

validateattributes(config.drho0, {'double'}, ...
    {'real', 'finite', 'scalar'}, ...
    mfilename, 'config.drho0');
validateattributes(config.drhoT, {'double'}, ...
    {'real', 'finite', 'scalar'}, ...
    mfilename, 'config.drhoT');

if config.drho0 >= 0
    error('mustBeProximityTransferConfig:InvalidDrho0', ...
        'drho0 必须为负值。');
end
if config.drhoT >= 0
    error('mustBeProximityTransferConfig:InvalidDrhoT', ...
        'drhoT 必须为负值。');
end

if config.drho0 >= config.drhoT
    error('mustBeProximityTransferConfig:InvalidSchedule', ...
        '需满足 drho0 < drhoT < 0，以形成减速型指数趋近。');
end

validateattributes(config.segment_samples, {'double'}, ...
    {'real', 'finite', 'scalar', 'integer', '>=', 2}, ...
    mfilename, 'config.segment_samples');
validateattributes(config.T_total_max, {'double'}, ...
    {'real', 'finite', 'scalar', 'positive'}, ...
    mfilename, 'config.T_total_max');

end
