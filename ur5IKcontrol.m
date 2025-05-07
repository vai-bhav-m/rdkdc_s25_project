function ur5IKcontrol(g_start,g_end,N,ur5)
current_theta = ur5.get_current_joints;
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
    pause(10);
end
end
