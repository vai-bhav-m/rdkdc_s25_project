% Initialize UR5 robot interface
ur5 = ur5_interface();

% Define start and end transformation matrices
g_start = [0 -1 0 0.25;
           -1 0 0 0.6;
            0 0 -1 0.22;
            0 0 0 1];

g_end =   [0 -1 0 0.40;
           -1 0 0 0.45;
            0 0 -1 0.22;
            0 0 0 1];

% Place visualization frames in RViz
tf_frame("base_link", "Start", g_start);
tf_frame("base_link", "End", g_end);

% Inverse kinematics for the start pose
candidate_thetas = ur5InvKin(g_start);

% Select configuration with highest manipulability
max_manip = -Inf;
best_idx = 1;

for i = 1:size(candidate_thetas, 2)
    J = ur5BodyJacobian(candidate_thetas(:, i));
    m = manipulability(J, 'invcond');
    if m > max_manip
        max_manip = m;
        best_idx = i;
    end
end

start_theta = candidate_thetas(:, best_idx);
ur5.move_joints(start_theta, 20);
pause(2);

% Move robot incrementally toward the goal using closest IK solution
current_theta = start_theta;
alpha = 0;
pos_start = g_start(1:3, 4);
pos_end = g_end(1:3, 4);

while norm(pos_start - pos_end) > 1e-6
    alpha = alpha + 0.01;
    interpolated_pos = pos_start + alpha * (pos_end - pos_start);
    g_interp = [g_start(1:3, 1:3), interpolated_pos; 0 0 0 1];

    current_theta = closest_IK(g_interp, current_theta);
    ur5.move_joints(current_theta, 10);
    pause(1);

    % Update position to check termination
    pos_start = interpolated_pos;
end

% --- Helper Function: Find closest IK solution to current joint state ---
function best_theta = closest_IK(g, q)
    thetas = ur5InvKin(g);
    [~, idx] = min(vecnorm(thetas - q));
    best_theta = thetas(:, idx);
end