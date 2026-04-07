function value_list = buildGradedList(breakpoints, segment_counts)
% buildGradedList 按分段变密度规则生成一维参数列表。
%
% 输入：
%   breakpoints     分段端点序列，要求严格单调。
%   segment_counts  各分段采样点数，长度比 breakpoints 少 1。
%
% 输出：
%   value_list      按原方向拼接后的参数列表，相邻分段公共端点只保留一次。

arguments
    breakpoints (1,:) double {mustBeReal, mustBeFinite, mustBeNonempty}
    segment_counts (1,:) double {mustBeReal, mustBeFinite, mustBeInteger, mustBePositive}
end

if numel(breakpoints) < 2
    error('buildGradedList:NotEnoughBreakpoints', ...
        'breakpoints 至少需要包含两个端点。');
end

if numel(segment_counts) ~= numel(breakpoints) - 1
    error('buildGradedList:SizeMismatch', ...
        'segment_counts 的长度必须比 breakpoints 少 1。');
end

break_diff = diff(breakpoints);
is_increasing = all(break_diff > 0);
is_decreasing = all(break_diff < 0);

if ~(is_increasing || is_decreasing)
    error('buildGradedList:NonMonotonicBreakpoints', ...
        'breakpoints 必须严格单调。');
end

if any(segment_counts < 2)
    error('buildGradedList:SegmentTooShort', ...
        '每一段至少需要 2 个采样点。');
end

value_list = [];

for i = 1:numel(segment_counts)
    segment_values = linspace(breakpoints(i), breakpoints(i + 1), segment_counts(i));

    if i > 1
        segment_values = segment_values(2:end);
    end

    value_list = [value_list, segment_values]; %#ok<AGROW>
end

end
