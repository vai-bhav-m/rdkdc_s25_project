%% Initialize UR5 robot interface
ur5 = ur5_interface();
pause(5);
ur5.switch_to_ros_control

%% Initialize safe config
theta_safe = [60; -80; 100; -120; -90; 40] * pi / 180;
ur5.move_joints(theta_safe, 15);
waitforbuttonpress;


% %% Manually set start
% ur5.switch_to_pendant_control;
% disp("Switched to pendant control")
% waitforbuttonpress;
% 
% q_start = ur5.get_current_joints();
% g_start = compute_FK_DH(q_start);
% g_end = g_start + [zeros(3) [0.025; 0.025; 0]; 0 0 0 1];
% 
% ur5.switch_to_ros_control
% disp("Switched to ros control")
% Define start and end transformation matrices
g_start = [0 -1 0 0.2;
           -1 0 0 0.5;
            0 0 -1 0.2;
            0 0 0 1];

g_end = [0 -1 0 0.40;
         -1 0 0 0.5;
          0 0 -1 0.2;
          0 0 0 1];

% Visualize frames in RViz
tf_frame("base_link", "Start", g_start);
tf_frame("base_link", "End", g_end);

%% Compute IK for start pose and choose best manipulability
% candidate_thetas = ur5InvKin(g_start);
% max_manip = -Inf;
% best_idx = 1;
% 
% for i = 1:size(candidate_thetas, 2)
%     J = ur5BodyJacobian(candidate_thetas(:, i));
%     m = manipulability(J, 'invcond');
%     if m > max_manip
%         max_manip = m;
%         best_idx = i;
%     end
% end
% 
% start_theta = candidate_thetas(:, best_idx);
start_theta = closest_IK(g_start, ur5.home);
ur5.move_joints(start_theta, 10);
pause(5);
disp("Moved to start position")
waitforbuttonpress;

%% RR
K = 2.5;
pause(5);
ur5JTcontrol(g_end,K,ur5)
%% Interpolate from start to end transformation
% current_theta = start_theta;
% N = 10;
% 
% R_start = g_start(1:3, 1:3);
% R_end = g_end(1:3, 1:3);
% pos_start = g_start(1:3, 4);
% pos_end = g_end(1:3, 4);
% 
% for i = 1:N
%     alpha = i / N;
%     pos_i = pos_start + alpha * (pos_end - pos_start);
%     R_i = interpolate_rotation(R_start, R_end, i, N);
%     g_interp = [g_start(1:3,1:3), pos_i; 0 0 0 1];
% 
%     current_theta = closest_IK(g_interp, current_theta);
%     ur5.move_joints(current_theta, 10);
%     pause(10);
% end

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