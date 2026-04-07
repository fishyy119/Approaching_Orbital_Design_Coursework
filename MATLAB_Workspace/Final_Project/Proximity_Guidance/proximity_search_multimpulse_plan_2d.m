function search = proximity_search_multimpulse_plan_2d( ...
    n, rel_r0, rel_v0, orbit, config)
% proximity_search_multimpulse_plan_2d 对近距离导引参数做四维网格搜索。
%
% 输入：
%   n        目标圆轨道角速度。
%   rel_r0   初始相对位置向量。
%   rel_v0   初始相对速度向量。
%   orbit    绕飞轨道参数结构体，包含 k 与 yc。
%   config   搜索配置结构体，包含 N、drho0、drhoT、切入相位网格
%            以及总转移时间上限。
%
% 输出：
%   search   搜索结果结构体，包含四维总脉冲张量、可行性标记、
%            三组二维归并参数面以及全局最优方案。
%
% 说明：
%   本函数对脉冲段数 N、起点调度导数 drho0、终点调度导数 drhoT 与
%   切入相位 theta 同时扫描。每个网格点调用多脉冲求解器，记录总 Δv
%   与转移时间，超过总转移时间上限的候选解直接剔除。随后再对非展示
%   维度做逐点最小化归并，以生成二维等高线图所需数据。

arguments
    n (1,1) double {mustBePositive}
    rel_r0 (2,1) double
    rel_v0 (2,1) double
    orbit (1,1) struct {mustBeProximityOrbitConfig}
    config (1,1) struct {mustBeProximitySearchConfig}
end

N_list = unique(config.N_list(:).');
drho0_list = unique(config.drho0_list(:).');
drhoT_list = unique(config.drhoT_list(:).');
theta_list = linspace(config.phase_bounds(1), config.phase_bounds(2), config.phase_count);

[rel_r_insert_list, rel_v_insert_list] = proximity_reference_orbit_state_2d( ...
    n, orbit, theta_list);

N_num = numel(N_list);
drho0_num = numel(drho0_list);
drhoT_num = numel(drhoT_list);
theta_num = numel(theta_list);

dv_total_tensor = nan(N_num, drho0_num, drhoT_num, theta_num);
dv_terminal_tensor = nan(N_num, drho0_num, drhoT_num, theta_num);
transfer_time_tensor = nan(N_num, drho0_num, drhoT_num, theta_num);
feasible_tensor = false(N_num, drho0_num, drhoT_num, theta_num);
time_limit_reject_count = 0;

valid_schedule_mask = reshape(drho0_list, [], 1) < reshape(drhoT_list, 1, []);
valid_schedule_pair_count = nnz(valid_schedule_mask);

if valid_schedule_pair_count == 0
    error('给定 drho0_list 与 drhoT_list 中不存在满足 drho0 < drhoT < 0 的参数对。');
end

best_total = inf;
best_plan = struct();
best_indices = struct('i_N', NaN, 'i_drho0', NaN, 'i_drhoT', NaN, 'i_theta', NaN);

transfer_config = struct();
transfer_config.N = N_list(1);
transfer_config.drho0 = drho0_list(1);
transfer_config.drhoT = drhoT_list(1);
transfer_config.segment_samples = config.segment_samples;
transfer_config.T_total_max = config.T_total_max;

for i_theta = 1:theta_num
    rel_rf = rel_r_insert_list(:, i_theta);
    rel_vf = rel_v_insert_list(:, i_theta);
    theta = theta_list(i_theta);

    for i_N = 1:N_num
        transfer_config.N = N_list(i_N);

        for i_drho0 = 1:drho0_num
            transfer_config.drho0 = drho0_list(i_drho0);

            for i_drhoT = 1:drhoT_num
                transfer_config.drhoT = drhoT_list(i_drhoT);

                if transfer_config.drho0 >= transfer_config.drhoT
                    continue;
                end

                plan = proximity_solve_line_multimpulse_2d( ...
                    n, rel_r0, rel_v0, rel_rf, rel_vf, transfer_config);

                if plan.time_limit_exceeded
                    time_limit_reject_count = time_limit_reject_count + 1;
                end

                if ~plan.is_feasible
                    continue;
                end

                plan.theta = theta;
                plan.orbit = orbit;

                feasible_tensor(i_N, i_drho0, i_drhoT, i_theta) = true;
                dv_total_tensor(i_N, i_drho0, i_drhoT, i_theta) = plan.dv_total;
                dv_terminal_tensor(i_N, i_drho0, i_drhoT, i_theta) = norm(plan.dv_terminal);
                transfer_time_tensor(i_N, i_drho0, i_drhoT, i_theta) = plan.T_total;

                if plan.dv_total < best_total
                    best_total = plan.dv_total;
                    best_plan = plan;
                    best_indices.i_N = i_N;
                    best_indices.i_drho0 = i_drho0;
                    best_indices.i_drhoT = i_drhoT;
                    best_indices.i_theta = i_theta;
                end
            end
        end
    end
end

if ~isfinite(best_total)
    error('给定四维搜索范围内未找到可行的多脉冲切入解。');
end

axis_defs = [ ...
    struct('name', 'N', 'values', N_list), ...
    struct('name', 'drho0', 'values', drho0_list), ...
    struct('name', 'drhoT', 'values', drhoT_list), ...
    struct('name', 'theta', 'values', theta_list)];

projection_drho = buildProjectedDvMap(dv_total_tensor, axis_defs, 2, 3);
projection_theta_drhoT = buildProjectedDvMap(dv_total_tensor, axis_defs, 4, 3);
projection_theta_drho0 = buildProjectedDvMap(dv_total_tensor, axis_defs, 4, 2);

search = struct();
search.N_list = N_list;
search.drho0_list = drho0_list;
search.drhoT_list = drhoT_list;
search.theta_list = theta_list;
search.T_total_max = config.T_total_max;
search.valid_schedule_mask = valid_schedule_mask;
search.rel_r_insert_list = rel_r_insert_list;
search.rel_v_insert_list = rel_v_insert_list;
search.dv_total_tensor = dv_total_tensor;
search.dv_terminal_tensor = dv_terminal_tensor;
search.transfer_time_tensor = transfer_time_tensor;
search.feasible_tensor = feasible_tensor;
search.total_case_count = N_num * drho0_num * drhoT_num * theta_num;
search.valid_schedule_case_count = N_num * valid_schedule_pair_count * theta_num;
search.feasible_case_count = nnz(feasible_tensor);
search.time_limit_reject_count = time_limit_reject_count;
search.projections = struct( ...
    'drho0_drhoT', projection_drho, ...
    'theta_drhoT', projection_theta_drhoT, ...
    'theta_drho0', projection_theta_drho0);
search.best_indices = best_indices;
search.best_plan = best_plan;

end

function projection = buildProjectedDvMap(dv_total_tensor, axis_defs, x_dim, y_dim)
% buildProjectedDvMap 对未展示维度做最小化归并，生成二维代价面。

x_list = axis_defs(x_dim).values;
y_list = axis_defs(y_dim).values;
collapse_dims = setdiff(1:4, [x_dim, y_dim], 'stable');

dv_map = nan(numel(y_list), numel(x_list));
optimizer_maps = cell(1, numel(collapse_dims));

for i = 1:numel(collapse_dims)
    optimizer_maps{i} = nan(numel(y_list), numel(x_list));
end

for iy = 1:numel(y_list)
    for ix = 1:numel(x_list)
        subs = {':', ':', ':', ':'};
        subs{x_dim} = ix;
        subs{y_dim} = iy;

        collapse_sizes = zeros(1, numel(collapse_dims));
        for i = 1:numel(collapse_dims)
            collapse_sizes(i) = numel(axis_defs(collapse_dims(i)).values);
        end

        slice = reshape(dv_total_tensor(subs{:}), collapse_sizes);
        slice_cost = slice;
        slice_cost(~isfinite(slice_cost)) = inf;

        [best_value, best_linear_index] = min(slice_cost(:));
        if ~isfinite(best_value)
            continue;
        end

        dv_map(iy, ix) = best_value;

        sub_index = cell(1, numel(collapse_dims));
        [sub_index{:}] = ind2sub(size(slice_cost), best_linear_index);

        for i = 1:numel(collapse_dims)
            optimizer_maps{i}(iy, ix) = axis_defs(collapse_dims(i)).values(sub_index{i});
        end
    end
end

projection = struct();
projection.x_list = x_list;
projection.y_list = y_list;
projection.dv_map = dv_map;

for i = 1:numel(collapse_dims)
    field_name = ['best_', axis_defs(collapse_dims(i)).name, '_map'];
    projection.(field_name) = optimizer_maps{i};
end

end

function mustBeProximityOrbitConfig(orbit)
% mustBeProximityOrbitConfig 校验绕飞轨道参数结构体。

if ~isstruct(orbit) || ~isscalar(orbit)
    error('mustBeProximityOrbitConfig:InvalidType', ...
        'orbit 必须为标量 struct。');
end

required_fields = {'k', 'yc'};
missing_fields = required_fields(~isfield(orbit, required_fields));
if ~isempty(missing_fields)
    error('mustBeProximityOrbitConfig:MissingField', ...
        'orbit 缺少字段：%s', strjoin(missing_fields, ', '));
end

validateattributes(orbit.k, {'double'}, ...
    {'real', 'finite', 'scalar', 'positive'}, ...
    mfilename, 'orbit.k');
validateattributes(orbit.yc, {'double'}, ...
    {'real', 'finite', 'scalar'}, ...
    mfilename, 'orbit.yc');

end

function mustBeProximitySearchConfig(config)
% mustBeProximitySearchConfig 校验四维参数搜索配置。

if ~isstruct(config) || ~isscalar(config)
    error('mustBeProximitySearchConfig:InvalidType', ...
        'config 必须为标量 struct。');
end

required_fields = {'N_list', 'drho0_list', 'drhoT_list', ...
    'phase_bounds', 'phase_count', 'segment_samples', 'T_total_max'};
missing_fields = required_fields(~isfield(config, required_fields));
if ~isempty(missing_fields)
    error('mustBeProximitySearchConfig:MissingField', ...
        'config 缺少字段：%s', strjoin(missing_fields, ', '));
end

validateattributes(config.N_list, {'double'}, ...
    {'real', 'finite', 'vector', 'nonempty', 'positive'}, ...
    mfilename, 'config.N_list');
mustBeInteger(config.N_list);

validateattributes(config.drho0_list, {'double'}, ...
    {'real', 'finite', 'vector', 'nonempty'}, ...
    mfilename, 'config.drho0_list');
validateattributes(config.drhoT_list, {'double'}, ...
    {'real', 'finite', 'vector', 'nonempty'}, ...
    mfilename, 'config.drhoT_list');

if any(config.drho0_list >= 0)
    error('mustBeProximitySearchConfig:InvalidDrho0List', ...
        'drho0_list 中所有值都必须为负值。');
end
if any(config.drhoT_list >= 0)
    error('mustBeProximitySearchConfig:InvalidDrhoTList', ...
        'drhoT_list 中所有值都必须为负值。');
end

validateattributes(config.phase_bounds, {'double'}, ...
    {'real', 'finite', 'vector', 'numel', 2}, ...
    mfilename, 'config.phase_bounds');
if ~(config.phase_bounds(1) < config.phase_bounds(2))
    error('mustBeProximitySearchConfig:InvalidPhaseBounds', ...
        'phase_bounds 必须满足下界小于上界。');
end

validateattributes(config.phase_count, {'double'}, ...
    {'real', 'finite', 'scalar', 'integer', '>=', 2}, ...
    mfilename, 'config.phase_count');
validateattributes(config.segment_samples, {'double'}, ...
    {'real', 'finite', 'scalar', 'integer', '>=', 2}, ...
    mfilename, 'config.segment_samples');
validateattributes(config.T_total_max, {'double'}, ...
    {'real', 'finite', 'scalar', 'positive'}, ...
    mfilename, 'config.T_total_max');

end
