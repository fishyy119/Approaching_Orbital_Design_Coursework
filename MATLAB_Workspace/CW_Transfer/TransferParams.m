%% 用于 Timpulse 脚本的便捷参数传递类
%
classdef TransferParams

properties
    % 轨道参数
    n(1, 1) double{mustBePositive} = 1

    % 初始状态
    r0(3, 1) double
    v0(3, 1) double

    % 目标状态
    rf(3, 1) double
    vf(3, 1) double

    % rho相关
    rho0(1, 1) double
    rhoT(1, 1) double
    drho0(1, 1) double
    drhoT(1, 1) double

    % 离散
    N(1, 1) double{mustBeInteger, mustBePositive} = 5

    % 类型
    path_type(1, 1) string{mustBeMember(path_type, ["line", "parabola"])} = "line"
    schedule_type(1, 1) string{mustBeMember(schedule_type, ["exp", "slow", "fast"])} = "exp"
end

end
