function g = compute_FK(theta)
% compute_FK Computes the forward kinematics for a 6-DOF UR5 robot
% Input:
%   theta - 1x6 vector of joint angles [theta1 ... theta6] in radians
% Output:
%   g     - 4x4 homogeneous transformation matrix from base to end-effector

    % DH parameters (UR5)
    d     = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823];
    a     = [0, -0.425, -0.39225, 0, 0, 0];
    alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];

    % Initialize total transformation as identity
    g = eye(4);

    % Multiply successive DH transformations
    for i = 1:6
        g = g * DH(a(i), alpha(i), d(i), theta(i));
    end
end

