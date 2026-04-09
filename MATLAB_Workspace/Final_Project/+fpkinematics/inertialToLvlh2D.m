function [rel_r_L, rel_v_L, info] = inertialToLvlh2D(r_t, v_t, r_c, v_c)
% inertialToLvlh2D 将二维惯性系状态转换为 LVLH 相对状态。

arguments
    r_t (2,1) double
    v_t (2,1) double
    r_c (2,1) double
    v_c (2,1) double
end

info = fpkinematics.lvlhFrame2D(r_t, v_t);
rel_r_L = info.C_I2L * (r_c - r_t);
rel_v_L = info.C_I2L * (v_c - v_t) - info.n * info.S * rel_r_L;

end
