function applyViridisColormap(N, is_reverse)
% APPLYVIRIDISCOLORMAP 设置 viridis 颜色图，可选反向显示

if nargin < 1 || isempty(N)
    N = 256;
end

if nargin < 2 || isempty(is_reverse)
    is_reverse = false;
end

if ~isempty(which('utils.viridis'))
    cmap = utils.viridis(N);
else
    cmap = parula(N);
end

% 根据需要对颜色图做上下翻转
if is_reverse
    cmap = flipud(cmap);
end

colormap(cmap);

end
