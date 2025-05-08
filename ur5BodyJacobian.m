function Jb = ur5BodyJacobian(joint_angles)
    % UR5 Link Parameters (in meters)
    base_offset = 0.089;       % H1
    shoulder_offset = 0.095;   % H2
    upper_arm_length = 0.425;  % L1
    forearm_length = 0.392;    % L2
    wrist_offset_1 = 0.109;    % W1
    wrist_offset_2 = 0.082;    % W2

    % Angular axes for each joint in the space frame
    joint_axes = [0, 0, 1;              % Joint 1
                  0, 1, 0;              % Joint 2
                  0, 1, 0;              % Joint 3
                  0, 1, 0;              % Joint 4
                  0, 0, -1;             % Joint 5
                  0, 1, 0];             % Joint 6

    % Linear part of twist vectors for each joint
    joint_velocities = [0, 0, 0;                                 % Joint 1
                        -base_offset, 0, 0;                      % Joint 2
                        -base_offset, 0, upper_arm_length;       % Joint 3
                        -base_offset, 0, upper_arm_length + forearm_length;    % Joint 4
                        -wrist_offset_1, upper_arm_length + forearm_length, 0; % Joint 5
                        shoulder_offset - base_offset, 0, upper_arm_length + forearm_length]; % Joint 6

    % Home configuration of the end-effector in the base frame
    gst0 = [-1, 0,  0, upper_arm_length + forearm_length;
             0, 0,  1, wrist_offset_1 + wrist_offset_2;
             0, 1,  0, base_offset - shoulder_offset;
             0, 0,  0, 1];

    % Initialize body Jacobian and end-effector transformation
    Jb = zeros(6, 6);
    current_transform = gst0;

    % Backward recursion to compute the body Jacobian
    for i = 6:-1:1
        omega = joint_axes(i, :)';
        v = joint_velocities(i, :)';
        twist_i = [v; omega];

        % Transform the twist into the body frame
        twist_body = AdjointInverse(current_transform) * twist_i;

        % Assign the column to the Jacobian
        Jb(:, i) = twist_body;

        % Update the transformation for the next iteration
        current_transform = TwistExponential(twist_i, joint_angles(i)) * current_transform;
    end
end

function adj_inv = AdjointInverse(transform)
    R = transform(1:3, 1:3);
    p = transform(1:3, 4);
    p_hat = [0, -p(3), p(2);
             p(3), 0, -p(1);
            -p(2), p(1), 0];
    adj_inv = [R', -R' * p_hat; zeros(3, 3), R'];
end

function transform = TwistExponential(twist, angle)
    v = twist(1:3);
    omega = twist(4:6);
    omega_hat = [0, -omega(3), omega(2);
                 omega(3), 0, -omega(1);
                -omega(2), omega(1), 0];

    if norm(omega) > 1e-10
        R = expm(omega_hat * angle);
        p = (eye(3) - R) * cross(omega, v) + omega * (omega' * v) * angle;
    else
        R = eye(3);
        p = v * angle;
    end

    transform = [R, p; 0, 0, 0, 1];
end
