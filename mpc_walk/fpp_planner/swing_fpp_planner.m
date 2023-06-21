function [fpp_swing_t, d_fpp_swing_t] = swing_fpp_planner(fpp_start, fpp_end, fpp_h, t_sw)
% calc single opti swing fpp based on local gait clock
% opti min energy during the swing phase
% t_sw is local clock for swing leg, 0~1

fpp_s_e = fpp_end - fpp_start;

x = fpp_start(1) + fpp_s_e(1)*(3*t_sw^2-2*t_sw^3);
d_x = fpp_s_e(1)*(6*t_sw-6*t_sw^2);
y = fpp_start(2) + fpp_s_e(2)*(3*t_sw^2-2*t_sw^3);
d_y = fpp_s_e(2)*(6*t_sw-6*t_sw^2);

z = 0;
d_z = 0;
if t_sw <= 0.5
    theta = 2*t_sw;
    z = fpp_start(3) + fpp_h*(3*theta^2-2*theta^3);
    d_z = fpp_h*(6*theta-6*theta^2);
else
    theta = 2*(t_sw - 0.5);
    h_a = fpp_h + fpp_start(3);
    z = h_a + (fpp_end(3)-h_a)*(3*theta^2-2*theta^3);
    d_z = (fpp_end(3)-h_a)*(6*theta-6*theta^2);
end

fpp_swing_t = [x; y; z];
d_fpp_swing_t = [d_x; d_y; d_z];

end

