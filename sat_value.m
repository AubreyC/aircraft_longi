function [x_out] = sat_value(x, x_min, x_max)
% Clamp the value between the max and min value

% Make sure x_min < x_max
if x_min > x_max
    x_min_real = x_max;
    x_max_real = x_min;
    x_max = x_max_real;
    x_min = x_min_real;
end

% COnstraint the value between x_min and x_max
x_out = max(x_min, min(x_max, x));

end

