function [x_out] = sat_value(x, x_min, x_max)
% Clamp the value between the max and min value

if x_min > x_max
    x_min_real = x_max;
    x_max_real = x_min;
    x_max = x_max_real;
    x_min = x_min_real;
end

x_out = max(x_min, min(x_max, x));

end

