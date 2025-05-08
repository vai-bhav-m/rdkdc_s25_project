function Jb = ur5BodyJacobian(joint_angles)
    % UR5 Link Parameters (in meters)
    D1 = 0.089;       % D1
    A2 = 0.425;  % A2
    A3 = 0.39225;    % A3
    D4 = 0.109;    % D4
    D5 = 0.095;   % D5
    D6 = 0.082;    % D6

    % Angular axes for each joint in the space frame
    joint_axes = [0, 0, 1;              % Joint 1
                  0, -1, 0;              % Joint 2
                  0, -1, 0;              % Joint 3
                  0, -1, 0;              % Joint 4
                  0, 0, -1;             % Joint 5
                  0, -1, 0];             % Joint 6
    
    % q values
    qs = [0, 0, 0;
        0, 0, D1;
        -A2, 0, D1;
        -A2 - A3, 0, D1;
        -A2 - A3, -D4, D1;
        -A2 - A3, -D4, D1-D5];
    
    % my j
    joint_velocities = zeros(6,3);
    for i=1:6
        joint_velocities(i,:) = cross(qs(i,:)',joint_axes(i,:)')';
    end

    % Linear part of twist vectors for each joint
    % joint_velocities = [0, 0, 0;                                 % Joint 1
    %                     -base_offset, 0, 0;                      % Joint 2
    %                     -base_offset, 0, upper_arm_length;       % Joint 3
    %                     -base_offset, 0, upper_arm_length + forearm_length;    % Joint 4
    %                     -wrist_offset_1, upper_arm_length + forearm_length, 0; % Joint 5
    %                     shoulder_offset - base_offset, 0, upper_arm_length + forearm_length] % Joint 6

    % Home configuration of the end-effector in the base frame
    gst0 = [1, 0,  0, -A2 - A3;
             0, 0,  1, -D4 - D6;
             0, -1,  0, D1 - D5;
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
