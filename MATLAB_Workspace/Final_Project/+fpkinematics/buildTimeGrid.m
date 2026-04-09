function time_grid = buildTimeGrid(duration, step_size)
% buildTimeGrid 构造包含起止端点的时间网格。

arguments
    duration (1,1) double {mustBeReal, mustBeFinite}
    step_size (1,1) double {mustBePositive}
end

if duration <= 0
    time_grid = 0;
    return;
end

sample_num = max(2, ceil(duration / step_size) + 1);
time_grid = linspace(0, duration, sample_num);

end
