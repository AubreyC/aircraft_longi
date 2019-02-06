function [alt_c_out] = compute_alt_command(alt,alt_c, max_climb_rate, delta_s)

delta_max_alt = sign(alt_c - alt)*abs(max_climb_rate)*delta_s;

delta_alt = alt_c - alt;

alt_c_out = alt_c;
if abs(delta_alt) > abs(delta_max_alt)
    
    alt_c_out = alt + delta_max_alt;
    
end

