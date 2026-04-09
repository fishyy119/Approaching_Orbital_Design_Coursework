function descriptor = matrix2xN(column_source)
% matrix2xN 构造 2 x N 矩阵描述符。

descriptor = struct( ...
    'kind', "matrix2xN", ...
    'column_source', column_source);

end
