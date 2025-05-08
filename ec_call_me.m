function ec_call_me(ur5)
    % nums = input("Enter a list of digits (0-9) in a vector format, e.g., [1 2 3]: ");
    % while ~isnumeric(nums) || any(nums < 0 | nums > 9 | floor(nums) ~= nums)
    %     disp("Invalid input. Please enter a vector of integers between 0 and 9.");
    %     nums = input("Enter a list of digits (0-9) in a vector format, e.g., [1 2 3]: ");
    % end
    nums = [0 7];

    % Switch to pendant control and select start point
    ur5.switch_to_pendant_control;
    pause(5);
    disp("Switched to pendant: select starting point");
    waitforbuttonpress;
    g_start = compute_FK_DH(ur5.get_current_joints);

    pause(2);
    disp("Switched to pendant: select end point");
    waitforbuttonpress;
    g_end = compute_FK_DH(ur5.get_current_joints);
    
    ur5.switch_to_ros_control;
    pause(2)
    theta_safe = [60; -80; 100; -120; -90; 40] * pi / 180;
    ur5.move_joints(theta_safe, 10);
    pause(10)

    % Average Z to keep movement in XY plane
    z_avg = (g_start(3,4) + g_end(3,4)) / 2;
    g_start(3,4) = z_avg;
    g_end(3,4) = z_avg;

    % Use same rotation for all waypoints
    R = g_start(1:3,1:3);
    g_p = @(p) [R, [p; z_avg]; 0 0 0 1];

    % Compute initial and final joint angles
    start_theta = closest_IK(g_start, ur5.home);
    end_theta = closest_IK(g_end, start_theta);
    
    ur5.move_joints(start_theta, 10);
    pause(5)

    % Get start and end XY coordinates
    t1 = g_start(1:2,4);
    t2 = g_end(1:2,4);
    l = norm(t1 - t2);
    dir = (t2 - t1) / l;
    pdir = [-dir(2); dir(1)];
    update_vec = 0.8 * l * pdir;

    curr_theta = start_theta;
    for j = 1:length(nums)
        wayp = get_waypoints(t1, t2, nums(j));
        curr_theta = closest_IK(g_p(wayp(:,1)), curr_theta);
        ur5.move_joints(curr_theta, 5);
        pause(5);
        for k = 2:size(wayp,2)
            p1 = wayp(:,k-1);
            p2 = wayp(:,k);
            ur5IKcontrol(g_p(p1), g_p(p2), 1, ur5);
            % ur5.move_joints(closest_IK(g_p(p2),curr_theta), 5);
            pause(5);
        end

        % Update path for next iteration
        t1 = t1 + update_vec;
        t2 = t2 + update_vec;
    

        % Move above next starting position
        g_n = g_p(t1);
        g_n(3,4) = g_n(3,4) + 0.03; % small Z offset
        curr_theta = closest_IK(g_n, curr_theta);
        ur5.move_joints(curr_theta, 5);
        pause(5);
    end

    disp("Hopefully done?");
end

function waypoints = get_waypoints(t1, t2, number)
    l = norm(t1 - t2);
    dir = (t2 - t1) / l;
    pdir = [-dir(2); dir(1)];
    rmov = l/2 * pdir;
    dmov = l/2 * dir;

    switch number
        case 0
            waypoints = [t1, t2, t2+rmov, t1+rmov, t1];
        case 1
            waypoints = [t1+rmov, t2+rmov];
        case 2
            waypoints = [t1, t1+rmov, t1+dmov+rmov, t1+dmov, t2, t2+rmov];
        case 3
            waypoints = [t1, t1+rmov, t1+dmov+rmov, t1+dmov, t1+dmov+rmov, t2+rmov, t2];
        case 4
            waypoints = [t1, t1+dmov, t1+dmov+rmov, t1+rmov, t2+rmov];
        case 5
            waypoints = [t1+rmov, t1, t1+dmov, t1+dmov+rmov, t2+rmov, t2];
        case 6
            waypoints = [t1+rmov, t1, t1+dmov, t1+dmov+rmov, t2+rmov, t2, t1+dmov];
        case 7
            waypoints = [t1, t1+rmov, t2+rmov];
        case 8
            waypoints = [t1, t2, t2+rmov, t1+rmov, t1, t1+dmov, t1+dmov+rmov];
        case 9
            waypoints = [t1+dmov+rmov, t1+dmov, t1, t1+rmov, t2+rmov, t2];
        otherwise
            error("Enter Valid Number");
    end
end
