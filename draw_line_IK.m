%% Initialize UR5 robot interface
% ur5 = ur5_interface();
% ur5.switch_to_ros_control

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
g_start = [0 -1 0 0.3;
           -1 0 0 0.5;
            0 0 -1 0.2;
            0 0 0 1];

g_end = [0 -1 0 0.40;
         -1 0 0 0.45;
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
disp("Moved to start position")
waitforbuttonpress;

%% Interpolate from start to end transformation
current_theta = start_theta;
N = 10;

R_start = g_start(1:3, 1:3);
R_end = g_end(1:3, 1:3);
pos_start = g_start(1:3, 4);
pos_end = g_end(1:3, 4);

for i = 1:N
    alpha = i / N;
    pos_i = pos_start + alpha * (pos_end - pos_start);
    R_i = interpolate_rotation(R_start, R_end, i, N);
    g_interp = [g_start(1:3,1:3), pos_i; 0 0 0 1];

    current_theta = closest_IK(g_interp, current_theta);
    ur5.move_joints(current_theta, 10);
    pause(10);
end

%% --- Helper Functions ---


