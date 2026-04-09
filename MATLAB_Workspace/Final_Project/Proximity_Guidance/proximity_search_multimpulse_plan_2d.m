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
%            若干二维归并参数面以及全局最优方案。
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
best_total_by_N = inf(1, N_num);
best_single_max_by_N = nan(1, N_num);
best_single_min_by_N = nan(1, N_num);
best_drho0_by_N = nan(1, N_num);
best_drhoT_by_N = nan(1, N_num);
best_theta_by_N = nan(1, N_num);

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

                if plan.dv_total < best_total_by_N(i_N)
                    dv_all_norm = [vecnorm(plan.dv_segments, 2, 1), norm(plan.dv_terminal)];
                    best_total_by_N(i_N) = plan.dv_total;
                    best_single_max_by_N(i_N) = max(dv_all_norm);
                    best_single_min_by_N(i_N) = min(dv_all_norm);
                    best_drho0_by_N(i_N) = plan.drho0;
                    best_drhoT_by_N(i_N) = plan.drhoT;
                    best_theta_by_N(i_N) = theta;
                end

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

projection_drho = buildProjectedMetricMap(dv_total_tensor, axis_defs, 2, 3);
projection_drho.dv_map = projection_drho.value_map;
projection_drho.theta_map = projection_drho.best_theta_map;
projection_drho.theta_deg_map = projection_drho.best_theta_map * 180 / pi;

projection_theta_drhoT = buildProjectedMetricMap(dv_total_tensor, axis_defs, 4, 3);
projection_theta_drhoT.dv_map = projection_theta_drhoT.value_map;

projection_theta_drho0 = buildProjectedMetricMap(dv_total_tensor, axis_defs, 4, 2);
projection_theta_drho0.dv_map = projection_theta_drho0.value_map;

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
search.optimal_by_N = struct( ...
    'N_list', N_list, ...
    'dv_total_list', replaceInfWithNaN(best_total_by_N), ...
    'dv_single_max_list', best_single_max_by_N, ...
    'dv_single_min_list', best_single_min_by_N, ...
    'drho0_list', best_drho0_by_N, ...
    'drhoT_list', best_drhoT_by_N, ...
    'theta_list', best_theta_by_N, ...
    'theta_deg_list', best_theta_by_N * 180 / pi);
search.optimal_dv_vs_time_by_N = buildOptimalDvVsTimeByN( ...
    N_list, transfer_time_tensor, dv_total_tensor);
search.best_indices = best_indices;
search.best_plan = best_plan;

end

function projection = buildProjectedMetricMap(metric_tensor, axis_defs, x_dim, y_dim)
% buildProjectedMetricMap 对未展示维度做最小化归并，生成二维指标面。

x_list = axis_defs(x_dim).values;
y_list = axis_defs(y_dim).values;
collapse_dims = setdiff(1:4, [x_dim, y_dim], 'stable');

value_map = nan(numel(y_list), numel(x_list));
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

        metric_slice = reshape(metric_tensor(subs{:}), collapse_sizes);
        slice_cost = metric_slice;
        slice_cost(~isfinite(slice_cost)) = inf;

        [best_value, best_linear_index] = min(slice_cost(:));
        if ~isfinite(best_value)
            continue;
        end

        value_map(iy, ix) = best_value;

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
projection.value_map = value_map;

for i = 1:numel(collapse_dims)
    field_name = ['best_', axis_defs(collapse_dims(i)).name, '_map'];
    projection.(field_name) = optimizer_maps{i};
end

end

function values = replaceInfWithNaN(values)
% replaceInfWithNaN 将未找到可行解的无穷值替换为 NaN。

values(~isfinite(values)) = nan;

end

function summary = buildOptimalDvVsTimeByN(N_list, transfer_time_tensor, dv_total_tensor)
% buildOptimalDvVsTimeByN 统计各 N 下随转移时间变化的最优总 Δv 前沿。

time_list_by_N = cell(1, numel(N_list));
dv_list_by_N = cell(1, numel(N_list));

for i_N = 1:numel(N_list)
    time_values = reshape(transfer_time_tensor(i_N, :, :, :), 1, []);
    dv_values = reshape(dv_total_tensor(i_N, :, :, :), 1, []);
    valid_mask = isfinite(time_values) & isfinite(dv_values);

    if ~any(valid_mask)
        time_list_by_N{i_N} = [];
        dv_list_by_N{i_N} = [];
        continue;
    end

    time_values = time_values(valid_mask);
    dv_values = dv_values(valid_mask);

    [time_sorted, sort_index] = sort(time_values);
    dv_sorted = dv_values(sort_index);
    [time_unique, ~, group_index] = unique(time_sorted, 'stable');
    dv_min_at_time = accumarray(group_index(:), dv_sorted(:), [], @min);
    dv_best_envelope = cummin(dv_min_at_time);

    time_list_by_N{i_N} = reshape(time_unique, 1, []);
    dv_list_by_N{i_N} = reshape(dv_best_envelope, 1, []);
end

summary = struct( ...
    'N_list', N_list, ...
    'transfer_time_list_by_N', {time_list_by_N}, ...
    'dv_total_list_by_N', {dv_list_by_N});

end

function mustBeProximityOrbitConfig(orbit)
% mustBeProximityOrbitConfig 校验绕飞轨道参数结构体。

schema = { ...
    'k', utils.schema.doubleScalar('positive'); ...
    'yc', utils.schema.doubleScalar()};

utils.schema.validateStruct(orbit, schema, 'orbit');

end

function mustBeProximitySearchConfig(config)
% mustBeProximitySearchConfig 校验四维参数搜索配置。

schema = { ...
    'N_list', utils.schema.doubleVector('finite', 'nonempty', 'integer', 'positive'); ...
    'drho0_list', utils.schema.doubleVector('finite', 'nonempty'); ...
    'drhoT_list', utils.schema.doubleVector('finite', 'nonempty'); ...
    'phase_bounds', utils.schema.doubleVector('finite', 'numel', 2); ...
    'phase_count', utils.schema.doubleScalar('integer', '>=', 2); ...
    'segment_samples', utils.schema.doubleScalar('integer', '>=', 2); ...
    'T_total_max', utils.schema.doubleScalar('positive')};

utils.schema.validateStruct(config, schema, 'config');

if any(config.drho0_list >= 0)
    error('mustBeProximitySearchConfig:InvalidDrho0List', ...
        'drho0_list 中所有值都必须为负值。');
end
if any(config.drhoT_list >= 0)
    error('mustBeProximitySearchConfig:InvalidDrhoTList', ...
        'drhoT_list 中所有值都必须为负值。');
end

if ~(config.phase_bounds(1) < config.phase_bounds(2))
    error('mustBeProximitySearchConfig:InvalidPhaseBounds', ...
        'phase_bounds 必须满足下界小于上界。');
end

end
