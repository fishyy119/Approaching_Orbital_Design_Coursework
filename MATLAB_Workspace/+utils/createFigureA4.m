function fig = createFigureA4(params)
% createFigureA4 使用结构体参数生成论文尺寸 figure。
%
% 输入：
%   params.Name         图窗名称，默认 ""
%   params.Width        图宽 cm，默认 18
%   params.AspectRatio  高宽比，默认 2/3

arguments
    params (1,1) struct = struct()
end

params = normalizeFigureParams(params);
height_cm = params.Width * params.AspectRatio;

fig = figure( ...
    'Name', params.Name, ...
    'NumberTitle', 'off', ...
    'Units', 'centimeters', ...
    'Position', [5, 5, params.Width, height_cm], ...
    'PaperUnits', 'centimeters', ...
    'PaperPosition', [0, 0, params.Width, height_cm]);

end

function params = normalizeFigureParams(params)
% normalizeFigureParams 合并默认值并校验 figure 参数。

defaults = struct( ...
    'Name', "", ...
    'Width', 18, ...
    'AspectRatio', 2 / 3);
params = mergeStructDefaults(params, defaults);

schema = { ...
    'Name', utils.schema.textScalar(); ...
    'Width', utils.schema.doubleScalar('positive'); ...
    'AspectRatio', utils.schema.doubleScalar('positive')};

utils.schema.validateStruct(params, schema, 'params');
params.Name = char(string(params.Name));

end

function value = mergeStructDefaults(value, defaults)
% mergeStructDefaults 用默认值补齐缺失字段。

if ~isstruct(value) || ~isscalar(value)
    error('utils.createFigureA4:InvalidParams', ...
        'params 必须为标量 struct。');
end

field_names = fieldnames(defaults);
for i = 1:numel(field_names)
    field_name = field_names{i};
    if ~isfield(value, field_name) || isempty(value.(field_name))
        value.(field_name) = defaults.(field_name);
    end
end

end
