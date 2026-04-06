%% 椭圆轨道拱点机动到目标圆轨道的速度增量参考
%
% 本脚本在共面、切向脉冲与相切转移的假设下，
% 计算给定初始椭圆轨道从近地点或远地点出发转移到目标圆轨道的参考 Δv。
% 结果可作为远程导引 Lambert 解的量级对照。
%
% 输出：
%   1. 近地点发起转移时的起点脉冲、末端圆化脉冲与总 Δv
%   2. 远地点发起转移时的起点脉冲、末端圆化脉冲与总 Δv

clear;
clc;

%% 常数
mu = 3.986004418e14; % m^3/s^2
Re = 6378137; % m

%% 任务场景参数
target = struct();
target.radius = Re + 1000e3; % 目标圆轨道半径

chaser = struct();
chaser.rp = Re + 400e3; % 初始椭圆轨道近地点半径
chaser.ra = Re + 600e3; % 初始椭圆轨道远地点半径
chaser.a = (chaser.rp + chaser.ra) / 2;
chaser.e = (chaser.ra - chaser.rp) / (chaser.ra + chaser.rp);

%% 参考计算
result = struct();
result.perigee_case = build_tangent_transfer_case( ...
    "近地点", chaser.rp, target.radius, chaser.a, mu);
result.apogee_case = build_tangent_transfer_case( ...
    "远地点", chaser.ra, target.radius, chaser.a, mu);

%% 输出
fprintf('初始椭圆轨道参数：\n');
fprintf('  rp = %.3f km\n', chaser.rp / 1e3);
fprintf('  ra = %.3f km\n', chaser.ra / 1e3);
fprintf('  a  = %.3f km\n', chaser.a / 1e3);
fprintf('  e  = %.6f\n', chaser.e);
fprintf('目标圆轨道半径：%.3f km\n\n', target.radius / 1e3);

print_transfer_case(result.perigee_case);
print_transfer_case(result.apogee_case);

%% 局部函数
function case_info = build_tangent_transfer_case( ...
    case_label, maneuver_radius, target_radius, initial_a, mu)
% build_tangent_transfer_case 计算拱点切向转移到目标圆轨道的参考 Δv。

initial_speed = sqrt(mu * (2 / maneuver_radius - 1 / initial_a));
target_speed = sqrt(mu / target_radius);

case_info = struct();
case_info.case_label = string(case_label);
case_info.maneuver_radius = maneuver_radius;
case_info.target_radius = target_radius;
case_info.initial_speed = initial_speed;
case_info.target_speed = target_speed;
case_info.transfer_a = nan;
case_info.transfer_e = nan;
case_info.transfer_time = 0;
case_info.departure_role = "";
case_info.arrival_role = "";
case_info.transfer_speed_departure = nan;
case_info.transfer_speed_arrival = nan;
case_info.dv_departure_signed = nan;
case_info.dv_departure = nan;
case_info.dv_arrival_signed = nan;
case_info.dv_arrival = nan;
case_info.dv_total = nan;

if maneuver_radius <= 0 || target_radius <= 0 || initial_a <= 0
    return;
end

if abs(target_radius - maneuver_radius) <= 1e-9 * max(target_radius, maneuver_radius)
    case_info.transfer_a = maneuver_radius;
    case_info.transfer_e = 0;
    case_info.transfer_speed_departure = target_speed;
    case_info.transfer_speed_arrival = target_speed;
    case_info.dv_departure_signed = target_speed - initial_speed;
    case_info.dv_departure = abs(case_info.dv_departure_signed);
    case_info.dv_arrival_signed = 0;
    case_info.dv_arrival = 0;
    case_info.dv_total = case_info.dv_departure;
    case_info.departure_role = "直接圆化";
    case_info.arrival_role = "直接圆化";
    return;
end

transfer_a = (maneuver_radius + target_radius) / 2;
transfer_e = abs(target_radius - maneuver_radius) / (target_radius + maneuver_radius);
transfer_speed_departure = sqrt(mu * (2 / maneuver_radius - 1 / transfer_a));
transfer_speed_arrival = sqrt(mu * (2 / target_radius - 1 / transfer_a));

case_info.transfer_a = transfer_a;
case_info.transfer_e = transfer_e;
case_info.transfer_time = pi * sqrt(transfer_a^3 / mu);
case_info.transfer_speed_departure = transfer_speed_departure;
case_info.transfer_speed_arrival = transfer_speed_arrival;
case_info.dv_departure_signed = transfer_speed_departure - initial_speed;
case_info.dv_departure = abs(case_info.dv_departure_signed);
case_info.dv_arrival_signed = target_speed - transfer_speed_arrival;
case_info.dv_arrival = abs(case_info.dv_arrival_signed);
case_info.dv_total = case_info.dv_departure + case_info.dv_arrival;

if target_radius > maneuver_radius
    case_info.departure_role = "转移轨道近地点";
    case_info.arrival_role = "转移轨道远地点";
else
    case_info.departure_role = "转移轨道远地点";
    case_info.arrival_role = "转移轨道近地点";
end

end

function print_transfer_case(case_info)
% print_transfer_case 输出单个拱点机动方案的参考结果。

fprintf('%s机动参考：\n', char(case_info.case_label));
fprintf('  机动点半径 = %.3f km\n', case_info.maneuver_radius / 1e3);
fprintf('  初始轨道速度 = %.6f m/s\n', case_info.initial_speed);
fprintf('  转移轨道起点速度 = %.6f m/s\n', case_info.transfer_speed_departure);
fprintf('  起点脉冲 Δv = %.6f m/s (signed = %.6f m/s)\n', ...
    case_info.dv_departure, case_info.dv_departure_signed);
fprintf('  转移轨道终点速度 = %.6f m/s\n', case_info.transfer_speed_arrival);
fprintf('  目标圆轨道速度 = %.6f m/s\n', case_info.target_speed);
fprintf('  末端圆化 Δv = %.6f m/s (signed = %.6f m/s)\n', ...
    case_info.dv_arrival, case_info.dv_arrival_signed);
fprintf('  总 Δv = %.6f m/s\n', case_info.dv_total);
fprintf('  转移半长轴 = %.3f km\n', case_info.transfer_a / 1e3);
fprintf('  转移偏心率 = %.6f\n', case_info.transfer_e);
fprintf('  转移飞行时间 = %.3f min\n', case_info.transfer_time / 60);
fprintf('  起点角色 = %s\n', char(case_info.departure_role));
fprintf('  终点角色 = %s\n\n', char(case_info.arrival_role));

end
