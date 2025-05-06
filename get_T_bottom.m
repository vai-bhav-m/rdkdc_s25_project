function t_bottom = get_T_bottom(s, e)
    % Midpoint
    mid = (s + e) / 2;

    % Vector from s to e
    v = e - s;

    % Perpendicular vector (rotate 90 degrees)
    v_perp = [-v(2); v(1)];

    % Normalize and scale the perpendicular vector
    d = norm(v);  % Distance to move from midpoint
    v_perp = v_perp / norm(v_perp);  % Unit vector

    % Compute bottom point on the perpendicular bisector
    t_bottom = mid + d * v_perp;
end
