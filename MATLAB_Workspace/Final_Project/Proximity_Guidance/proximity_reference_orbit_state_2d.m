function [rel_r_ref, rel_v_ref] = proximity_reference_orbit_state_2d(n, orbit, theta)
% proximity_reference_orbit_state_2d 生成平面绕飞轨道在 LVLH 系中的状态。
%
% 轨道形式：
%   x = k * sin(theta)
%   y = 2 * k * cos(theta) + yc

arguments
    n (1,1) double {mustBePositive}
    orbit (1,1) struct {mustBeProximityReferenceOrbit}
    theta (1,:) double
end

rel_r_ref = [orbit.k * sin(theta); ...
    2 * orbit.k * cos(theta) + orbit.yc];
rel_v_ref = [orbit.k * n * cos(theta); ...
    -2 * orbit.k * n * sin(theta)];

end

function mustBeProximityReferenceOrbit(orbit)
% mustBeProximityReferenceOrbit 校验绕飞轨道参数结构体。

if ~isstruct(orbit) || ~isscalar(orbit)
    error('mustBeProximityReferenceOrbit:InvalidType', ...
        'orbit 必须为标量 struct。');
end

required_fields = {'k', 'yc'};
missing_fields = required_fields(~isfield(orbit, required_fields));
if ~isempty(missing_fields)
    error('mustBeProximityReferenceOrbit:MissingField', ...
        'orbit 缺少字段：%s', strjoin(missing_fields, ', '));
end

validateattributes(orbit.k, {'double'}, ...
    {'real', 'finite', 'scalar', 'positive'}, ...
    mfilename, 'orbit.k');
validateattributes(orbit.yc, {'double'}, ...
    {'real', 'finite', 'scalar'}, ...
    mfilename, 'orbit.yc');

end
