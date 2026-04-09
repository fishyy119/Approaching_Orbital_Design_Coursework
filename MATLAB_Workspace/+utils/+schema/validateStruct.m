function validateStruct(struct_value, schema, struct_name)
% validateStruct 按声明式 schema 校验标量结构体。
%
% 用法：
%   utils.schema.validateStruct(struct_value, schema, struct_name)
%
% 输入：
%   struct_value   待校验的标量结构体。
%   schema         N x 2 单元格数组，每行定义一个字段校验规则：
%                    第 1 列：字段名；
%                    第 2 列：校验器，可取以下三类之一：
%                      1. []：
%                         仅要求字段存在，不进一步校验字段值。
%                      2. 函数句柄：
%                         用于自定义校验逻辑。
%                      3. utils.schema.* 描述符：
%                         用于声明常见校验规则，例如数值标量、
%                         文本、时间向量、2 x N 矩阵和嵌套结构体。
%   struct_name    该结构体在错误信息中的显示名，例如 'config'、
%                  'simulation.wait'。它只用于拼接报错路径，
%                  不参与具体校验逻辑。
%
% 自定义函数句柄支持以下签名之一：
%   validator(value)
%   validator(value, value_name)
%   validator(value, value_name, struct_value)
%
% 常用描述符：
%   utils.schema.doubleScalar(...)
%   utils.schema.doubleVector(...)
%   utils.schema.textScalar()
%   utils.schema.timeVector(expected_count)
%   utils.schema.matrix2xN(column_source)
%   utils.schema.vector2x1()
%   utils.schema.nested(sub_schema)
%
% 示例：
%   wait_schema = { ...
%       'time', utils.schema.timeVector(); ...
%       'chaser_pos', utils.schema.matrix2xN('time'); ...
%       'label', utils.schema.textScalar()};
%
%   utils.schema.validateStruct(wait_block, wait_schema, 'simulation.wait')

validate_schema_definition(schema);

struct_name_text = char(string(struct_name));
field_name_list = string(schema(:, 1));
field_name_cell = cellstr(field_name_list);

if ~isstruct(struct_value) || ~isscalar(struct_value)
    error('utils.schema.validateStruct:InvalidType', ...
        '%s 必须为标量 struct。', struct_name_text);
end

missing_fields = field_name_list(~isfield(struct_value, field_name_cell));
if ~isempty(missing_fields)
    error('utils.schema.validateStruct:MissingField', ...
        '%s 缺少字段：%s', ...
        struct_name_text, strjoin(cellstr(missing_fields), ', '));
end

for i = 1:size(schema, 1)
    field_name = field_name_cell{i};
    validator = schema{i, 2};

    if isempty(validator)
        continue;
    end

    value_name = sprintf('%s.%s', struct_name_text, field_name);
    invoke_validator(validator, struct_value.(field_name), value_name, struct_value);
end

end

function validate_schema_definition(schema)
% validate_schema_definition 校验 schema 自身格式。

if ~iscell(schema) || size(schema, 2) ~= 2
    error('utils.schema.validateStruct:InvalidSchema', ...
        'schema 必须为 N x 2 单元格数组。');
end

for i = 1:size(schema, 1)
    field_name = schema{i, 1};
    validator = schema{i, 2};

    is_valid_name = ischar(field_name) || (isstring(field_name) && isscalar(field_name));
    if ~is_valid_name
        error('utils.schema.validateStruct:InvalidSchema', ...
            'schema 第 %d 行的字段名必须为字符向量或标量字符串。', i);
    end

    if isempty(validator)
        continue;
    end

    if isa(validator, 'function_handle')
        continue;
    end

    if isSchemaDescriptor(validator)
        validateDescriptorDefinition(validator, i);
        continue;
    end

    error('utils.schema.validateStruct:InvalidSchema', ...
        'schema 第 %d 行的校验器必须为函数句柄、utils.schema 描述符或空数组。', i);
end

end

function invoke_validator(validator, value, value_name, struct_value)
% invoke_validator 按函数签名分派 schema 校验器。

if isSchemaDescriptor(validator)
    invokeDescriptor(validator, value, value_name, struct_value);
    return;
end

validator_nargin = nargin(validator);

if validator_nargin == 1
    validator(value);
elseif validator_nargin == 2
    validator(value, value_name);
else
    validator(value, value_name, struct_value);
end

end

function is_descriptor = isSchemaDescriptor(validator)
% isSchemaDescriptor 判断 schema 项是否为描述符结构体。

is_descriptor = isstruct(validator) && isscalar(validator) ...
    && isfield(validator, 'kind');

end

function validateDescriptorDefinition(descriptor, row_index)
% validateDescriptorDefinition 校验描述符自身格式。

kind = getDescriptorKind(descriptor, row_index);

switch kind
    case "validateattributes"
        validateDescriptorCellField(descriptor, 'classes', row_index);
        validateDescriptorCellField(descriptor, 'attributes', row_index);
    case "timevector"
        validateReferenceField(descriptor, 'expected_count', row_index);
    case "matrix2xn"
        validateReferenceField(descriptor, 'column_source', row_index);
    case {"vector2x1", "textscalar"}
        return;
    case "nested"
        if ~isfield(descriptor, 'schema')
            error('utils.schema.validateStruct:InvalidSchema', ...
                'schema 第 %d 行的 nested 描述符缺少 schema 字段。', row_index);
        end

        validate_schema_definition(descriptor.schema);
    otherwise
        error('utils.schema.validateStruct:InvalidSchema', ...
            'schema 第 %d 行使用了未知描述符类型 %s。', ...
            row_index, char(kind));
end

end

function kind = getDescriptorKind(descriptor, row_index)
% getDescriptorKind 读取并标准化描述符类型名称。

kind = string(descriptor.kind);
if ~isscalar(kind) || strlength(kind) == 0
    error('utils.schema.validateStruct:InvalidSchema', ...
        'schema 第 %d 行的描述符 kind 必须为非空标量字符串。', row_index);
end

kind = lower(kind);

end

function validateDescriptorCellField(descriptor, field_name, row_index)
% validateDescriptorCellField 校验描述符中的单元格字段。

if ~isfield(descriptor, field_name) || ~iscell(descriptor.(field_name))
    error('utils.schema.validateStruct:InvalidSchema', ...
        'schema 第 %d 行的描述符字段 %s 必须为单元格数组。', ...
        row_index, field_name);
end

end

function validateReferenceField(descriptor, field_name, row_index)
% validateReferenceField 校验描述符中的引用字段。

if ~isfield(descriptor, field_name)
    return;
end

reference = descriptor.(field_name);
if isempty(reference)
    return;
end

is_valid_numeric = isnumeric(reference) && isscalar(reference);
is_valid_text = ischar(reference) || (isstring(reference) && isscalar(reference));

if ~is_valid_numeric && ~is_valid_text
    error('utils.schema.validateStruct:InvalidSchema', ...
        'schema 第 %d 行的描述符字段 %s 必须为标量数值、字符向量、标量字符串或空数组。', ...
        row_index, field_name);
end

end

function invokeDescriptor(descriptor, value, value_name, struct_value)
% invokeDescriptor 按描述符类型分派校验逻辑。

kind = getDescriptorKind(descriptor, NaN);

switch kind
    case "validateattributes"
        validateattributes(value, descriptor.classes, descriptor.attributes, ...
            'utils.schema.validateStruct', value_name);
    case "timevector"
        expected_count = [];
        if isfield(descriptor, 'expected_count')
            expected_count = resolveCountReference( ...
                descriptor.expected_count, struct_value, value_name, 'expected_count');
        end

        validateTimeVectorLocal(value, value_name, expected_count);
    case "matrix2xn"
        column_count = resolveCountReference( ...
            descriptor.column_source, struct_value, value_name, 'column_source');
        validateMatrix2xNLocal(value, column_count, value_name);
    case "vector2x1"
        validateVector2x1Local(value, value_name);
    case "textscalar"
        validateTextScalarLocal(value, value_name);
    case "nested"
        utils.schema.validateStruct(value, descriptor.schema, value_name);
    otherwise
        error('utils.schema.validateStruct:InvalidSchema', ...
            '字段 %s 使用了未知描述符类型 %s。', ...
            value_name, char(kind));
end

end

function resolved_count = resolveCountReference(reference, struct_value, value_name, field_name)
% resolveCountReference 解析描述符中的长度引用。

if isempty(reference)
    resolved_count = [];
    return;
end

if isnumeric(reference) && isscalar(reference)
    resolved_count = reference;
    return;
end

reference_name = char(string(reference));
if ~isfield(struct_value, reference_name)
    error('utils.schema.validateStruct:InvalidSchemaReference', ...
        '%s 的 schema 引用了不存在的字段 %s（%s）。', ...
        value_name, reference_name, field_name);
end

resolved_count = numel(struct_value.(reference_name));

end

function element_count = validateTimeVectorLocal(value, value_name, expected_count)
% validateTimeVectorLocal 校验时间向量。

if nargin < 3
    expected_count = [];
end

validateattributes(value, {'double'}, {'real', 'finite', 'vector', 'nonempty'}, ...
    'utils.schema.validateStruct', value_name);

element_count = numel(value);
if ~isempty(expected_count) && element_count ~= expected_count
    error('utils.schema.validateStruct:SizeMismatch', ...
        '%s 的元素个数应为 %d，当前为 %d。', ...
        value_name, expected_count, element_count);
end

end

function validateMatrix2xNLocal(value, column_count, value_name)
% validateMatrix2xNLocal 校验二维矩阵为 2 x N。

validateattributes(value, {'double'}, {'real', 'finite', '2d'}, ...
    'utils.schema.validateStruct', value_name);

if size(value, 1) ~= 2
    error('utils.schema.validateStruct:SizeMismatch', ...
        '%s 的行数应为 2，当前为 %d。', ...
        value_name, size(value, 1));
end

if size(value, 2) ~= column_count
    error('utils.schema.validateStruct:SizeMismatch', ...
        '%s 的列数应为 %d，当前为 %d。', ...
        value_name, column_count, size(value, 2));
end

end

function validateVector2x1Local(value, value_name)
% validateVector2x1Local 校验二维列向量。

validateattributes(value, {'double'}, {'real', 'finite', 'size', [2, 1]}, ...
    'utils.schema.validateStruct', value_name);

end

function validateTextScalarLocal(value, value_name)
% validateTextScalarLocal 校验文本字段为标量字符串或行字符向量。

is_valid_string = isstring(value) && isscalar(value);
is_valid_char = ischar(value) && (isrow(value) || isempty(value));

if ~(is_valid_string || is_valid_char)
    error('utils.schema.validateStruct:InvalidText', ...
        '%s 必须为标量字符串或行字符向量。', value_name);
end

end
