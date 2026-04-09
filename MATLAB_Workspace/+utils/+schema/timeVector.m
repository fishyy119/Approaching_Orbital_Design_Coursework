function descriptor = timeVector(expected_count)
% timeVector 构造时间向量描述符。

if nargin < 1
    expected_count = [];
end

descriptor = struct( ...
    'kind', "timeVector", ...
    'expected_count', expected_count);

end
