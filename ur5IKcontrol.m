function ur5IKcontrol(g_start,g_end,N,ur5)
current_theta = ur5InvKin(g_start);
R_start = g_start(1:3, 1:3);
R_end = g_end(1:3, 1:3);
pos_start = g_start(1:3, 4);
pos_end = g_end(1:3, 4);

for i = 1:N
    alpha = i / N;
    pos_i = pos_start + alpha * (pos_end - pos_start);
    R_i = interpolate_rotation(R_start, R_end, i, N);
    g_interp = [R_i, pos_i; 0 0 0 1];

    current_theta = closest_IK(g_interp, current_theta);
    ur5.move_joints(current_theta, 10);
    pause(5);
end
end

%% --- Helper Functions ---

function best_theta = closest_IK(g, q)
    % Returns IK solution closest to current joint state q
    thetas = ur5InvKin(g);
    [~, idx] = min(vecnorm(thetas - q));
    best_theta = thetas(:, idx);
end

function R_i = interpolate_rotation(R_in, R_out, i, N)
    % Interpolates between two rotation matrices using the Lie algebra
    R = R_out * R_in';
    [theta, omega] = rotation_to_axis_angle(R);
    omega_scaled = omega * (theta * i / N);
    R_i = expm(SKEW3(omega_scaled)) * R_in;
end

function [theta, omega] = rotation_to_axis_angle(R)
    % Converts rotation matrix to axis-angle representation
    theta = acos((trace(R) - 1) / 2);
    if theta == 0
        omega = [0; 0; 0];
    elseif abs(theta - pi) < 1e-6
        % Special case: rotation by pi
        [V, D] = eig(R);
        omega = V(:, abs(diag(D) - 1) < 1e-6);
        omega = omega / norm(omega);
    else
        omega = 0.5 / sin(theta) * [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)];
    end
end