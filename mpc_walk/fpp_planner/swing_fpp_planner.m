function [fpp_swing_t] = swing_fpp_planner(fpp_start, fpp_end, fpp_h, t_sw)
% calc single opti swing fpp based on local gait clock

fpp_s_e = fpp_end - fpp_start;

x = fpp_start(1) + fpp_s_e(1)*(3*t_sw^2-2*t_sw^3);
d_x = fpp_s_e(1)*(6*t_sw-6*t_sw^2);
y = fpp_start(2) + fpp_s_e(2)*(3*t_sw^2-2*t_sw^3);
d_y = fpp_s_e(2)*(6*t_sw-6*t_sw^2);

fpp_swing_t = 0;

end
