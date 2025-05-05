function g = compute_FK_DH(theta)
    % Add pi offset to the first joint angle
    theta(1) = theta(1) + pi;

    % DH parameters for UR5 (link offsets and lengths)
    d     = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823];   % Link offsets
    a     = [0, -0.425, -0.39225, 0, 0, 0];                 % Link lengths
    alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];                   % Link twists

    % Initialize the transformation matrix as identity matrix
    g = eye(4);

    % Multiply successive DH transformation matrices
    for i = 1:6
        g = g * DH(a(i), alpha(i), d(i), theta(i));  % Apply DH transformation for each joint
    end
end


