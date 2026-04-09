function frame = lvlhFrame2D(r_t, v_t)
% lvlhFrame2D 由目标惯性系状态构造二维 LVLH 坐标基与旋转信息。

arguments
    r_t (2,1) double
    v_t (2,1) double
end

r_norm = norm(r_t);
h_z = r_t(1) * v_t(2) - r_t(2) * v_t(1);

if r_norm <= 0
    error('fpkinematics.lvlhFrame2D:DegeneratePosition', ...
        '目标位置向量退化，无法构造 LVLH 基。');
end
if abs(h_z) <= eps(max([1, r_norm * norm(v_t)]))
    error('fpkinematics.lvlhFrame2D:DegenerateAngularMomentum', ...
        '目标角动量过小，无法构造稳定的 LVLH 基。');
end

e_x = r_t / r_norm;
e_y = sign(h_z) * [-e_x(2); e_x(1)];

frame = struct();
frame.C_L2I = [e_x, e_y];
frame.C_I2L = frame.C_L2I.';
frame.n = h_z / r_norm^2;
frame.S = [0, -1; 1, 0];
frame.h_z = h_z;

end
