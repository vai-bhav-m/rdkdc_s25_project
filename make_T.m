function make_T(control_type, K)
%% Initialize UR5 robot interface
ur5 = ur5_interface();
pause(5);
ur5.switch_to_ros_control

%% Initialize safe config
theta_safe = [60; -80; 100; -120; -90; 40] * pi / 180;
ur5.move_joints(theta_safe, 15);
pause(10)

g_start = [0 -1 0 0.3;
           -1 0 0 0.5;
            0 0 -1 0.2;
            0 0 0 1];

g_end = [0 -1 0 0.40;
         -1 0 0 0.45;
          0 0 -1 0.2;
          0 0 0 1];

% Making z coordinate the same
z_avg = (g_start(3,4) + g_end(3,4)) / 2;
g_start(3,4) = z_avg;
g_end(3,4) = z_avg;

% Visualize frames in RViz
tf_frame("base_link", "Start", g_start);
tf_frame("base_link", "End", g_end);

% Move to starting position
start_theta = closest_IK(g_start, ur5.home);
end_theta = closest_IK(g_end, start_theta);
ur5.move_joints(start_theta, 10);
pause(5);
disp("Moved to start position. Click a button to continue")
waitforbuttonpress;

% Apply control from start to end
if control_type == "IK"
    ur5IKcontrol(g_start,g_end,K,ur5);
elseif control_type == "RR"
    ur5RRcontrol(g_end,K,ur5);
    ur5.move_joints(end_theta, 10);
    pause(10);
elseif control_type == "JT"
    ur5JTcontrol(g_end,K,ur5);
    ur5.move_joints(end_theta, 10);
    pause(10);
else
    error('INVALID Control request, enter one of {IK, RR, JT}');
end

pause(10)
% Computing mid-point
R_mid = interpolate_rotation(g_start(1:3,1:3), g_end(1:3,1:3), 1, 2);
t_mid = (g_start(1:3,4) + g_end(1:3,4)) / 2;
g_mid = [R_mid t_mid; 0 0 0 1];

% Computing bottom
R_bottom = R_mid;
t_bottom = [get_T_bottom(g_start(1:2,4), g_end(1:2,4)); g_end(3,4)];
g_bottom = [R_bottom t_bottom; 0 0 0 1];

tf_frame("base_link", "Bottom", g_bottom);


disp("Done with first leg. Moving to home and attempting second leg.  Click a button to continue")
ur5.move_joints(theta_safe, 15);
pause(5)
waitforbuttonpress;

% Move to middle position
mid_theta = closest_IK(g_mid, ur5.home);
end_theta = closest_IK(g_bottom, mid_theta);
ur5.move_joints(mid_theta, 10);
pause(5);
disp("Moved to mid position. Click a button to continue")
waitforbuttonpress;

% Apply control from start to end
if control_type == "IK"
    ur5IKcontrol(g_mid,g_bottom,K,ur5);
elseif control_type == "RR"
    ur5RRcontrol(g_bottom,K,ur5);
    ur5.move_joints(end_theta, 10);
    pause(10);
elseif control_type == "JT"
    ur5JTcontrol(g_bottom,K,ur5);
    ur5.move_joints(end_theta, 10);
    pause(10);
else
    error('INVALID Control request, enter one of {IK, RR, JT}');
end

pause(10)

disp("DONE")


