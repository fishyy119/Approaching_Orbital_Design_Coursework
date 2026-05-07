%% STK AdvCAT 接近告警时间段绘图
%
% 本脚本读取 STK AdvCAT 导出的 CLOSE APPROACH ENCOUNTER WARNINGS 报告，
% 抽取每条告警的最近接近时刻与告警持续时间，并生成一张告警点图。

close all;
clear;
clc;
utils.setDefaultGraphics();

%% 输入路径
script_dir = fileparts(mfilename('fullpath'));
report_file = fullfile(script_dir, 'AdvCAT1_Encounter_Warnings.txt');

if ~isfile(report_file)
    error('plot_advcat_encounter_warnings:MissingReport', ...
        '未找到 STK 接近告警报告：%s', report_file);
end

%% 读取并解析 STK 告警报告
[encounters, report_info] = parseAdvcatEncounterWarnings(report_file);

fprintf('STK 接近告警报告：%s\n', report_file);
fprintf('筛选阈值 = %.3f km\n', report_info.screening_threshold_km);
fprintf('唯一接近告警事件数 = %d\n', height(encounters));

[min_range_km, min_idx] = min(encounters.range_km);
fprintf('最近接近事件：SAT#1 %s, SAT#2 %s, TCA = %s, 最近距离 = %.3f km\n', ...
    char(encounters.sat1_id(min_idx)), char(encounters.sat2_id(min_idx)), ...
    datestr(encounters.tca(min_idx), 'yyyy-mm-dd HH:MM:SS.FFF'), min_range_km);

%% 图：有告警最近时刻与持续时间
fig_tca_duration = plotEncounterTcaDuration(encounters, report_info);

%% 汇总输出结构体
advcat_encounter_result = struct();
advcat_encounter_result.report_file = report_file;
advcat_encounter_result.report_info = report_info;
advcat_encounter_result.encounters = encounters;
advcat_encounter_result.figures = struct( ...
    'tca_duration', fig_tca_duration);

%% 本地函数
function [encounters, report_info] = parseAdvcatEncounterWarnings(report_file)
% parseAdvcatEncounterWarnings 解析 STK AdvCAT 接近告警文本报告。

text_raw = fileread(report_file);

threshold_tokens = regexp(text_raw, ...
    'Screening Threshold\s*=\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][+-]?\d+)?)\s*km', ...
    'tokens', 'once');
if isempty(threshold_tokens)
    screening_threshold_km = nan;
else
    screening_threshold_km = str2double(threshold_tokens{1});
end

[run_start, run_end] = parseRunTimeSpan(text_raw);

blocks = regexp(text_raw, '\*{10,}', 'split');
records = defaultEncounterRecord();
records = records([]);

for i_block = 1:numel(blocks)
    block = blocks{i_block};

    if ~contains(block, 'CLOSEST APPROACH')
        continue;
    end

    record = parseEncounterBlock(block);
    if ismissing(record.sat1_id) || isnat(record.tca) || ~isfinite(record.range_km)
        continue;
    end

    records(end + 1) = record; %#ok<AGROW>
end

if isempty(records)
    error('plot_advcat_encounter_warnings:NoEncounterWarnings', ...
        '报告中未解析到 CLOSEST APPROACH 接近告警事件。');
end

encounters = struct2table(records);
encounters = removeDuplicateEncounters(encounters);
encounters = sortrows(encounters, 'tca');
encounters.duration_s = seconds(encounters.exit_time - encounters.entry_time);
encounters.tca.Format = 'MM-dd HH:mm:ss.SSS';
encounters.entry_time.Format = 'MM-dd HH:mm:ss.SSS';
encounters.exit_time.Format = 'MM-dd HH:mm:ss.SSS';

report_info = struct();
report_info.screening_threshold_km = screening_threshold_km;
report_info.run_start = run_start;
report_info.run_end = run_end;
report_info.record_count_raw = numel(records);
report_info.record_count_unique = height(encounters);

end

function [run_start, run_end] = parseRunTimeSpan(text_raw)
% parseRunTimeSpan 读取 STK 报告给出的仿真时间范围。

run_start = NaT;
run_end = NaT;
time_expr = '(\d{1,2}\s+[A-Za-z]{3}\s+\d{4})\s+(\d{2}:\d{2}:\d{2}(?:\.\d+)?)';

start_tokens = regexp(text_raw, ['Start:\s*', time_expr], 'tokens', 'once');
if ~isempty(start_tokens)
    run_start = parseStkDatetime(start_tokens{1}, start_tokens{2});
end

end_tokens = regexp(text_raw, ['End:\s*', time_expr], 'tokens', 'once');
if ~isempty(end_tokens)
    run_end = parseStkDatetime(end_tokens{1}, end_tokens{2});
end

end

function record = defaultEncounterRecord()
% defaultEncounterRecord 构造单条接近告警记录的默认值。

record = struct( ...
    'sat1_id', string(missing), ...
    'sat1_name', string(missing), ...
    'sat2_id', string(missing), ...
    'sat2_name', string(missing), ...
    'entry_time', NaT, ...
    'entry_range_km', nan, ...
    'tca', NaT, ...
    'range_km', nan, ...
    'exit_time', NaT, ...
    'exit_range_km', nan, ...
    'rel_pos_u_km', nan, ...
    'rel_pos_v_km', nan, ...
    'rel_pos_w_km', nan, ...
    'rel_speed_kmps', nan, ...
    'rel_vel_u_kmps', nan, ...
    'rel_vel_v_kmps', nan, ...
    'rel_vel_w_kmps', nan, ...
    'approach_angle_deg', nan);

end

function record = parseEncounterBlock(block)
% parseEncounterBlock 解析单个 AdvCAT 接近告警块。

record = defaultEncounterRecord();

[record.sat1_id, record.sat1_name] = parseSatelliteHeader(block, 1);
[record.sat2_id, record.sat2_name] = parseSatelliteHeader(block, 2);

[record.entry_range_km, record.entry_time] = parseRangeTimeLine(block, 'THRESHOLD ENTRY');
[record.range_km, record.tca] = parseRangeTimeLine(block, 'CLOSEST APPROACH');
[record.exit_range_km, record.exit_time] = parseRangeTimeLine(block, 'THRESHOLD EXIT');

record.rel_pos_u_km = parseComponent(block, 'Normal', 'U', 'km');
record.rel_pos_v_km = parseComponent(block, 'Tangential', 'V', 'km');
record.rel_pos_w_km = parseComponent(block, 'Cross-Track', 'W', 'km');

record.rel_speed_kmps = parseLabeledMagnitude(block, ...
    'Relative VELOCITY at Time of Closest Approach', 'km/s');
record.rel_vel_u_kmps = parseComponent(block, 'Normal', 'U', 'km/s');
record.rel_vel_v_kmps = parseComponent(block, 'Tangential', 'V', 'km/s');
record.rel_vel_w_kmps = parseComponent(block, 'Cross-Track', 'W', 'km/s');
record.approach_angle_deg = parseApproachAngle(block);

end

function [sat_id, sat_name] = parseSatelliteHeader(block, sat_index)
% parseSatelliteHeader 读取 Satellite #1/#2 行。

expr = sprintf('Satellite #%d:\\s*(\\S+)\\s+([^\\r\\n]+)', sat_index);
tokens = regexp(block, expr, 'tokens', 'once');

if isempty(tokens)
    sat_id = string(missing);
    sat_name = string(missing);
    return;
end

sat_id = string(strtrim(tokens{1}));
sat_name = string(strtrim(tokens{2}));

end

function [range_km, event_time] = parseRangeTimeLine(block, event_label)
% parseRangeTimeLine 读取 ENTRY、TCA 或 EXIT 行。

num_expr = numberRegexp();
expr = ['(?m)^\s*', num_expr, ...
    '\s+(\d{1,2}\s+[A-Za-z]{3}\s+\d{4})', ...
    '\s+(\d{2}:\d{2}:\d{2}(?:\.\d+)?)\s+', ...
    regexptranslate('escape', event_label)];
tokens = regexp(block, expr, 'tokens', 'once');

if isempty(tokens)
    range_km = nan;
    event_time = NaT;
    return;
end

range_km = str2double(tokens{1});
event_time = parseStkDatetime(tokens{2}, tokens{3});

end

function value = parseComponent(block, component_name, component_symbol, unit_text)
% parseComponent 读取 U/V/W 方向的第一列分量，即 SAT#2 w.r.t. SAT#1。

num_expr = numberRegexp();
expr = ['(?m)^\s*', regexptranslate('escape', component_name), ...
    '\s+\[', component_symbol, '\]:\s*', num_expr, ...
    '\s+', regexptranslate('escape', unit_text), '(?=\s)'];
tokens = regexp(block, expr, 'tokens', 'once');

if isempty(tokens)
    value = nan;
else
    value = str2double(tokens{1});
end

end

function value = parseLabeledMagnitude(block, label_text, unit_text)
% parseLabeledMagnitude 读取带括号的标量量值。

num_expr = numberRegexp();
expr = [regexptranslate('escape', label_text), ...
    ':\s*\(\s*', num_expr, '\s*', regexptranslate('escape', unit_text), '\s*\)'];
tokens = regexp(block, expr, 'tokens', 'once');

if isempty(tokens)
    value = nan;
else
    value = str2double(tokens{1});
end

end

function angle_deg = parseApproachAngle(block)
% parseApproachAngle 读取接近角，单位为 deg。

num_expr = numberRegexp();
tokens = regexp(block, ['Approach Angle\s*=\s*', num_expr, '\s*deg'], ...
    'tokens', 'once');

if isempty(tokens)
    angle_deg = nan;
else
    angle_deg = str2double(tokens{1});
end

end

function event_time = parseStkDatetime(date_text, time_text)
% parseStkDatetime 将 STK 英文月份时间转换为 MATLAB datetime。

time_string = [strtrim(date_text), ' ', strtrim(time_text)];

try
    event_time = datetime(time_string, ...
        'InputFormat', 'dd MMM yyyy HH:mm:ss.SSS', ...
        'Locale', 'en_US');
catch
    event_time = datetime(time_string, ...
        'InputFormat', 'dd MMM yyyy HH:mm:ss', ...
        'Locale', 'en_US');
end

end

function num_expr = numberRegexp()
% numberRegexp 返回通用浮点数正则表达式。

num_expr = '([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][+-]?\d+)?)';

end

function encounters = removeDuplicateEncounters(encounters)
% removeDuplicateEncounters 去除 STK 报告中可能重复列出的同一告警。

tca_key = string(datestr(encounters.tca, 'yyyy-mm-dd HH:MM:SS.FFF'));
key = encounters.sat1_id + "|" + encounters.sat2_id + "|" + ...
    tca_key + "|" + compose('%.6f', encounters.range_km);
[~, unique_idx] = unique(key, 'stable');
encounters = encounters(unique_idx, :);

end

function fig = plotEncounterTcaDuration(encounters, report_info)
% plotEncounterTcaDuration 绘制有告警的 TCA 时刻与持续时间。

[~, order] = sort(encounters.tca, 'ascend');
encounters = encounters(order, :);
warning_id = (1:height(encounters))';

fig = utils.createFigureA4(struct( ...
    'Name', 'STK AdvCAT 接近告警时刻', ...
    'Width', 8, ...
    'AspectRatio', 0.62));
ax = axes('Parent', fig);
hold(ax, 'on');
grid(ax, 'on');
box(ax, 'on');

marker_size = 52;
scatter(ax, encounters.tca, warning_id, marker_size, encounters.range_km, ...
    'filled', ...
    'MarkerEdgeColor', 'k', ...
    'DisplayName', utils.formatMixedFontText('最近接近时刻'));

set(ax, ...
    'YTick', warning_id, ...
    'YTickLabel', cellstr(compose('%d', warning_id)), ...
    'YDir', 'reverse');

if ~isnat(report_info.run_start) && ~isnat(report_info.run_end)
    x_min = report_info.run_start;
    x_max = report_info.run_end;
else
    x_margin = minutes(30);
    x_min = min(encounters.entry_time) - x_margin;
    x_max = max(encounters.exit_time) + x_margin;
end
xlim(ax, [x_min, x_max]);
x_tick_count = 6;
ax.XTick = x_min + seconds(linspace(0, seconds(x_max - x_min), x_tick_count));


utils.applyViridisColormap(256, true);
cb = colorbar(ax);
ylabel(cb, utils.formatMixedFontText('最近距离 (km)'));

xlabel(ax, utils.formatMixedFontText('时间'));
ylabel(ax, utils.formatMixedFontText('告警编号'));
ylim(ax, [0.5, height(encounters) + 0.5]);
xtickformat(ax, 'MM-dd HH:mm');
utils.applyPlotTitle(ax, utils.formatMixedFontText('STK AdvCAT 接近告警时刻与持续时间'));
legend(ax, 'Location', 'best');

time_span = x_max - x_min;
label_offset = 0.006 * time_span;
if label_offset < seconds(5000)
    label_offset = seconds(5000);
end
split_idx = ceil(height(encounters) / 2);

for i = 1:height(encounters)
    if i <= split_idx
        label_time = encounters.tca(i) + label_offset;
        horizontal_alignment = 'left';
    else
        label_time = encounters.tca(i) - label_offset;
        horizontal_alignment = 'right';
    end

    text(ax, label_time, warning_id(i), ...
        sprintf('%.3f s', encounters.duration_s(i)), ...
        'VerticalAlignment', 'middle', ...
        'HorizontalAlignment', horizontal_alignment, ...
        'FontSize', 8, ...
        'Clipping', 'on');
end

end
