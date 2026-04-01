%% 霍曼转移2
%
% 计算航天器从椭圆轨道到圆轨道的霍曼转移所需的速度增量。
% 遍历近远、远近、近近、远远四种情况，计算相应的Δv。

clc;
clear;

%%
for c = 1:4
    ocase(c)
end

%%
function ocase(tcase)
R_e = 6378; % km
mu_e = 398600; % km^3/s^2
h_11 = 400; % km
h_12 = 600; % km
h_21 = 800; % km
h_22 = 1000; % km

a_1 = R_e + (h_11 + h_12) / 2;
a_2 = R_e + (h_21 + h_22) / 2;

switch tcase
    case 1
        tip = '近远';
        t_1 = R_e + h_11;
        t_2 = R_e + h_22;
    case 2
        tip = '远近';
        t_1 = R_e + h_12;
        t_2 = R_e + h_21;
    case 3
        tip = '近近';
        t_1 = R_e + h_11;
        t_2 = R_e + h_21;
    case 4
        tip = '远远';
        t_1 = R_e + h_12;
        t_2 = R_e + h_22;
end

a_e = (t_1 + t_2) / 2;

v_1 = ra2v(t_1, a_1, mu_e);
v_2 = ra2v(t_2, a_2, mu_e);

v_12 = ra2v(t_1, a_e, mu_e);
v_21 = ra2v(t_2, a_e, mu_e);

d1 = v_12 - v_1;
d2 = v_2 - v_21;
fprintf('------------------------\n%s\n', tip)
fprintf('dv_1 = %f\n', d1)
fprintf('dv_2 = %f\n', d2)
fprintf('sum = %f\n', abs(d1) + abs(d2))
end
