clc;
clear;

%%
R_e = 6378; % km
mu_e = 398600; % km^3/s^2
h_1 = 400; % km
h_2 = 600; % km

a_1 = R_e + h_1;
a_2 = R_e + h_2;
a_e = (a_1 + a_2) / 2;

v_1 = ra2v(a_1, a_1, mu_e);
v_2 = ra2v(a_2, a_2, mu_e);

v_12 = ra2v(a_1, a_e, mu_e);
v_21 = ra2v(a_2, a_e, mu_e);

d1 = v_12 - v_1;
d2 = v_2 - v_21;
fprintf('dv_1 = %f\n', d1)
fprintf('dv_2 = %f\n', d2)
fprintf('sum = %f\n', abs(d1)+abs(d2))
