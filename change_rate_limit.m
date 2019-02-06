function [x_limited] = change_rate_limit(x_init,x_desired, max_rate, delta_s)
% Limit the change of a value according to a change rate

% Delta max according to max rate:
delta_x_max = sign(x_desired - x_init)*abs(max_rate)*delta_s;

% Delta desired
delta_x = x_desired - x_init;

% Limit with maximal delta
if abs(delta_x) > abs(delta_x_max)
    x_limited = x_init + delta_x_max;
else
    x_limited = x_desired;
    
end

