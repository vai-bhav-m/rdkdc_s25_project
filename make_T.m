function make_T(ur5, control_type, K)
%% Hardcoded start and end
% g_start = [0.0531   -0.9969   -0.0575    0.3270;
%    -0.9962   -0.0489   -0.0716    0.5805;
%     0.0686    0.0611   -0.9958    0.1450;
%          0         0         0    1.0000];
% 
% g_end = [0.0531   -0.9969   -0.0575    0.4;
%    -0.9962   -0.0489   -0.0716    0.620;
%     0.0686    0.0611   -0.9958    0.1450;
%          0         0         0    1.0000];


%% Custom inputs
ur5.switch_to_pendant_control;
pause(5)
disp("Switched to pendant: select starting point")
waitforbuttonpress

g_start = compute_FK_DH(ur5.get_current_joints);

pause(2)

disp("Switched to pendant: select end point")
waitforbuttonpress

g_end = compute_FK_DH(ur5.get_current_joints);

% Making z coordinate the same
z_avg = (g_start(3,4) + g_end(3,4)) / 2;
g_start(3,4) = z_avg;
g_end(3,4) = z_avg;

%% Initialize safe config
ur5.switch_to_ros_control;
disp("Switched to ros control")
pause(5)

theta_safe = [60; -80; 100; -120; -90; 40] * pi / 180;
ur5.move_joints(theta_safe, 10);
pause(10)


 % g_pen = [1 0 0 0; 0 1 0 -0.049; 0 0 1 -0.12228; 0 0 0 1 ]  
 % g_start = g_start*g_pen
 % g_end = g_end*g_pen

% Visualize frames in RViz
tf_frame("base_link", "Start", g_start);
tf_frame("base_link", "End", g_end);

% Move to starting position
start_theta = closest_IK(g_start, ur5.home);
end_theta = closest_IK(g_end, start_theta);

% Safety check
if (safety(start_theta,g_end(3,4)-0.02) == false)
    error("Choose safe starting point")
elseif (safety(end_theta,g_end(3,4)-0.02) == false)
    error("Choose safe end point")
end

% Move to starting point
ur5.move_joints(start_theta, 10);
pause(5);
disp("Moved to start position. Click to start control")
waitforbuttonpress;

% Apply control from start to end
disp("Starting control")
if control_type == "IK"
    ur5IKcontrol(g_start,g_end,K,ur5);
elseif control_type == "RR"
    ur5RRcontrol(g_end,K,ur5);
    ur5.move_joints(end_theta, 5);
    pause(10);
elseif control_type == "JT"
    ur5JTcontrol(g_end,K,ur5);
    ur5.move_joints(end_theta, 5);
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


disp("Done with first leg. Preparing for second leg.")
ur5.move_joints(theta_safe, 15);
pause(5)

% Move to middle position
mid_theta = closest_IK(g_mid, ur5.home);
end_theta = closest_IK(g_bottom, mid_theta);
ur5.move_joints(mid_theta, 10);
pause(5);
disp("Moved to mid position. Click to start control")
waitforbuttonpress;

% Apply control from start to end
disp("Starting control")
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

disp("DONE WITH DRAWING")
ur5.move_joints(theta_safe, 15);
pause(5)


