function [x_out] = sat_value(x, x_min, x_max)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

x_out = max(x_min, min(x_max, x));

end

