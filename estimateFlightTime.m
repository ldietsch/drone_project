function T = estimateFlightTime(d,u_max)
    % Assumes that the quad accelerates with half the max acc. for half the distance
    % Time for half the distance
    t_half = sqrt(d/(u_max/2));
    v_half = u_max/2*t_half;
    
    T = t_half + d/2*(1/v_half);
end