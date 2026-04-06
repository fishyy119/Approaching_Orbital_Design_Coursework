function E = solve_kepler_ellipse(M, e)
% solve_kepler_ellipse 用牛顿迭代求解椭圆轨道的偏近点角。

M_wrapped = atan2(sin(M), cos(M));

if e < 0.8
    E = M_wrapped;
else
    E = pi * sign(M_wrapped);
    if abs(M_wrapped) < 1e-12
        E = pi;
    end
end

for k = 1:50
    f = E - e * sin(E) - M_wrapped;
    fp = 1 - e * cos(E);
    dE = -f / fp;
    E = E + dE;

    if abs(dE) < 1e-12
        break;
    end
end

end
