function [r_c, v_c, info] = lvlhToInertial2D(r_t, v_t, rel_r_L, rel_v_L)
% lvlhToInertial2D 将二维 LVLH 相对状态转换为惯性系状态。

arguments
    r_t (2,1) double
    v_t (2,1) double
    rel_r_L (2,1) double
    rel_v_L (2,1) double
end

info = fpkinematics.lvlhFrame2D(r_t, v_t);
r_c = r_t + info.C_L2I * rel_r_L;
v_c = v_t + info.C_L2I * (rel_v_L + info.n * info.S * rel_r_L);

end
