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