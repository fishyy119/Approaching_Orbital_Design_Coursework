function animation_info = render_remote_guidance_animation_2d(simulation, config)
% render_remote_guidance_animation_2d 渲染远程导引数值转移动画，并按需导出 GIF。
%
% 输入：
%   simulation        远程导引数值仿真结果结构体。
%   config            动画播放与导出配置结构体。
%
% 输出：
%   animation_info    动画帧数、时长和导出结果信息。

arguments
    simulation (1,1) struct {mustBeRemoteGuidanceSimulation}
    config (1,1) struct {mustBeRemoteGuidanceAnimationConfig} = struct( ...
        'frame_count', 240, ...
        'pause_seconds', 0.02, ...
        'gif_delay', 0.04, ...
        'save_gif', false, ...
        'gif_path', '')
end

gif_path_text = string(config.gif_path);

fig = utils.createFigureA4(struct( ...
    'Name', '最优 Lambert 转移动画', ...
    'Width', 18, ...
    'AspectRatio', 0.80));
ax = axes('Parent', fig);
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
box(ax, 'on');

theta = linspace(0, 2 * pi, 400);
fill(ax, simulation.Re * cos(theta) / 1e3, simulation.Re * sin(theta) / 1e3, ...
    [0.92, 0.94, 0.98], ...
    'EdgeColor', [0.55, 0.60, 0.70], ...
    'HandleVisibility', 'off');
plot(ax, simulation.reference.target_pos(1, :) / 1e3, ...
    simulation.reference.target_pos(2, :) / 1e3, ...
    ':', 'Color', [0.80, 0.25, 0.25], 'LineWidth', 1.0, ...
    'HandleVisibility', 'off');
plot(ax, simulation.reference.chaser_pos(1, :) / 1e3, ...
    simulation.reference.chaser_pos(2, :) / 1e3, ...
    ':', 'Color', [0.55, 0.55, 0.55], 'LineWidth', 1.0, ...
    'HandleVisibility', 'off');

trail_wait = animatedline(ax, 'Color', [0.10, 0.35, 0.85], ...
    'LineStyle', '--', 'LineWidth', 1.8, ...
    'DisplayName', utils.formatMixedFontText('等待段'));
trail_transfer = animatedline(ax, 'Color', [0.10, 0.35, 0.85], ...
    'LineStyle', '-', 'LineWidth', 2.0, ...
    'DisplayName', utils.formatMixedFontText('转移段'));
trail_target = animatedline(ax, 'Color', [0.85, 0.20, 0.20], ...
    'LineStyle', '-.', 'LineWidth', 1.4, ...
    'DisplayName', utils.formatMixedFontText('目标飞行段'));

marker_departure_fixed = plot(ax, ...
    simulation.events.departure(1) / 1e3, ...
    simulation.events.departure(2) / 1e3, ...
    's', ...
    'MarkerSize', 6.5, ...
    'MarkerFaceColor', [0.98, 0.75, 0.20], ...
    'MarkerEdgeColor', 'k', ...
    'DisplayName', utils.formatMixedFontText('转移起点'));
marker_capture_fixed = plot(ax, ...
    simulation.events.arrival_capture(1) / 1e3, ...
    simulation.events.arrival_capture(2) / 1e3, ...
    'd', ...
    'MarkerSize', 6.5, ...
    'MarkerFaceColor', [0.15, 0.65, 0.35], ...
    'MarkerEdgeColor', 'k', ...
    'DisplayName', utils.formatMixedFontText('最终捕获点'));
marker_chaser = plot(ax, NaN, NaN, 'o', ...
    'MarkerSize', 7, ...
    'MarkerFaceColor', [0.10, 0.35, 0.85], ...
    'MarkerEdgeColor', 'k', ...
    'DisplayName', utils.formatMixedFontText('跟踪器'));
marker_target = plot(ax, NaN, NaN, 'o', ...
    'MarkerSize', 7, ...
    'MarkerFaceColor', [0.85, 0.20, 0.20], ...
    'MarkerEdgeColor', 'k', ...
    'DisplayName', utils.formatMixedFontText('目标飞行器'));

xlim(ax, simulation.axis_limit_km * [-1, 1]);
ylim(ax, simulation.axis_limit_km * [-1, 1]);
xlabel(ax, utils.formatMixedFontText('惯性系 {\itx} (km)'));
ylabel(ax, utils.formatMixedFontText('惯性系 {\ity} (km)'));
utils.applyPlotTitle(ax, utils.formatMixedFontText('最优 Lambert 转移动画'));
legend(ax, 'Location', 'bestoutside');

t_anim = linspace(0, simulation.mission.time(end), config.frame_count);
r_chaser_anim = interpolate_trajectory_2d( ...
    simulation.mission.time, simulation.mission.chaser_pos, t_anim);
r_target_anim = interpolate_trajectory_2d( ...
    simulation.mission.time, simulation.mission.target_pos, t_anim);

if config.save_gif && strlength(gif_path_text) > 0
    gif_dir = fileparts(gif_path_text);
    if strlength(string(gif_dir)) > 0 && ~isfolder(gif_dir)
        mkdir(gif_dir);
    end
end

if config.save_gif && strlength(gif_path_text) > 0 && isfile(gif_path_text)
    delete(gif_path_text);
end

transfer_started = false;

for i = 1:numel(t_anim)
    t_now = t_anim(i);

    addpoints(trail_target, r_target_anim(1, i) / 1e3, r_target_anim(2, i) / 1e3);

    if t_now <= simulation.best.td + 1e-9
        addpoints(trail_wait, r_chaser_anim(1, i) / 1e3, r_chaser_anim(2, i) / 1e3);
    else
        if ~transfer_started
            addpoints(trail_transfer, ...
                simulation.events.departure(1) / 1e3, ...
                simulation.events.departure(2) / 1e3);
            transfer_started = true;
        end

        addpoints(trail_transfer, ...
            r_chaser_anim(1, i) / 1e3, r_chaser_anim(2, i) / 1e3);
    end

    marker_chaser.XData = r_chaser_anim(1, i) / 1e3;
    marker_chaser.YData = r_chaser_anim(2, i) / 1e3;
    marker_target.XData = r_target_anim(1, i) / 1e3;
    marker_target.YData = r_target_anim(2, i) / 1e3;

    if t_now <= simulation.best.td + 1e-9
        phase_text = '等待段';
    else
        phase_text = '转移段';
    end

    time_text = sprintf('%.1f', t_now / 60);
    title_text = ['最优 Lambert 转移动画，', phase_text, '，{\itt} = ', time_text, ' min'];
    utils.applyPlotTitle(ax, utils.formatMixedFontText(title_text));

    drawnow;

    if config.save_gif && strlength(gif_path_text) > 0
        frame = getframe(fig);
        [frame_image, frame_map] = rgb2ind(frame2im(frame), 256);

        if i == 1
            imwrite(frame_image, frame_map, gif_path_text, 'gif', ...
                'LoopCount', inf, 'DelayTime', config.gif_delay);
        else
            imwrite(frame_image, frame_map, gif_path_text, 'gif', ...
                'WriteMode', 'append', 'DelayTime', config.gif_delay);
        end
    end

    pause(config.pause_seconds);
end

animation_info = struct();
animation_info.frame_count = config.frame_count;
animation_info.duration = simulation.mission.time(end);
animation_info.save_gif = config.save_gif;
animation_info.gif_path = "";

if config.save_gif && strlength(gif_path_text) > 0
    animation_info.gif_path = gif_path_text;
    fprintf('转移动画 GIF 已保存到：%s\n', char(gif_path_text));
end

end

function mustBeRemoteGuidanceAnimationConfig(config)
% mustBeRemoteGuidanceAnimationConfig 校验动画配置结构体。

schema = { ...
    'frame_count', utils.schema.doubleScalar('integer', 'positive'); ...
    'pause_seconds', utils.schema.doubleScalar('nonnegative'); ...
    'gif_delay', utils.schema.doubleScalar('nonnegative'); ...
    'save_gif', utils.schema.logicalScalar(); ...
    'gif_path', utils.schema.textScalar()};

utils.schema.validateStruct(config, schema, 'config');

end

function position_query = interpolate_trajectory_2d(time_nodes, position_nodes, time_query)
% interpolate_trajectory_2d 对二维轨迹做分量插值。

position_query = zeros(2, numel(time_query));
position_query(1, :) = interp1(time_nodes, position_nodes(1, :), time_query, 'pchip');
position_query(2, :) = interp1(time_nodes, position_nodes(2, :), time_query, 'pchip');

end
