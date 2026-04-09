function descriptor = nested(schema)
% nested 构造嵌套结构体 schema 描述符。

descriptor = struct( ...
    'kind', "nested", ...
    'schema', {schema});

end
