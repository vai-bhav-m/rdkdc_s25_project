function is_safe = safety_constraint_check(theta)
    % Step 1: Compute rotated + translated frame
    Rz = [cosd(-30), -sind(-30), 0;
          sind(-30),  cosd(-30), 0;
               0,          0,     1];
    y_axis_rot = Rz * [0; 1; 0];  % rotated Y-axis = plane normal
    x_axis_rot = Rz * [1; 0; 0];
    z_axis_rot = Rz * [0; 0; 1];
    normal = y_axis_rot / norm(y_axis_rot)

    P_plane = [-0.200, 0, 0]';

    % Initialize as safe
    is_safe = true;

    % UR5 DH parameters
    g = eye(4);
    d     = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823];
    a     = [0, -0.425, -0.39225, 0, 0, 0];
    alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];

    theta(1) = theta(1) + pi;

    joint_positions = zeros(6, 3);

    fprintf('--- Checking Joint Frames ---\n');

    for i = 1:6
        g = g * DH(a(i), alpha(i), d(i), theta(i));
        pos = g(1:3, 4);  % Extract [x, y, z]
        joint_positions(i, :) = pos;
        
        % Compute signed distance to plane
        signed_dist = normal' *(pos - P_plane);

        fprintf('Joint %d position: X=%.2f, Y=%.2f, Z=%.2f | SignedDist=%.2f | Z=%.2f\n', ...
            i, pos(1), pos(2), pos(3), signed_dist, pos(3));

        if signed_dist < 0 || pos(3) < 0
            fprintf(' Joint %d FAILED constraint (negative side of y=0 plane or Z < 0)\n', i);
            is_safe = false;
        end
    end

    if is_safe
        fprintf(' All joints passed constraints.\n');
    end

    % =====================
    % PLOTTING SECTION
    % =====================
    fig = figure('Name', sprintf('Safety Check Visualization'), 'NumberTitle', 'off');
    hold on; grid on; axis equal;
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    title('Plane y=0 in Rotated+Translated Frame with Joint Positions');

    % Define meshgrid limits
    all_x = [joint_positions(:,1); P_plane(1)];
    all_y = [joint_positions(:,2); P_plane(2)];
    x_range = linspace(min(all_x)-200, max(all_x)+200, 10);
    y_range = linspace(min(all_y)-200, max(all_y)+200, 10);

    % Robust plane plotting
    abs_normal = abs(normal);
    [~, max_idx] = max(abs_normal);

    switch max_idx
        case 1  % solve for X
            [Y, Z] = meshgrid(y_range, linspace(0, 1200, 10));
            X_plane = (-normal(2)*(Y - P_plane(2)) - normal(3)*(Z - P_plane(3))) / normal(1) + P_plane(1);
            surf(X_plane, Y, Z, 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'FaceColor', [0.6 0.8 1]);
        case 2  % solve for Y
            [X, Z] = meshgrid(x_range, linspace(0, 1200, 10));
            Y_plane = (-normal(1)*(X - P_plane(1)) - normal(3)*(Z - P_plane(3))) / normal(2) + P_plane(2);
            surf(X, Y_plane, Z, 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'FaceColor', [0.6 0.8 1]);
        case 3  % solve for Z
            [X, Y] = meshgrid(x_range, y_range);
            Z_plane = (-normal(1)*(X - P_plane(1)) - normal(2)*(Y - P_plane(2))) / normal(3) + P_plane(3);
            surf(X, Y, Z_plane, 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'FaceColor', [0.6 0.8 1]);
    end

    % Plot Z = 0 ground plane
    [X_ground, Y_ground] = meshgrid(x_range, y_range);
    Z_ground = zeros(size(X_ground));
    surf(X_ground, Y_ground, Z_ground, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'FaceColor', [0.9 0.7 0.7]);

    % Plot joint positions
    scatter3(joint_positions(:,1), joint_positions(:,2), joint_positions(:,3), 100, 'filled', 'm');

    % Label joints
    for i = 1:6
        text(joint_positions(i,1), joint_positions(i,2), joint_positions(i,3), ...
             sprintf('J%d', i), 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
    end

    % Plot base frame arrows at origin
    quiver3(0, 0, 0, 100, 0, 0, 'r', 'LineWidth', 2);
    quiver3(0, 0, 0, 0, 100, 0, 'g', 'LineWidth', 2);
    quiver3(0, 0, 0, 0, 0, 100, 'b', 'LineWidth', 2);

    % Plot rotated+translated frame arrows
    quiver3(P_plane(1), P_plane(2), P_plane(3), 100*x_axis_rot(1), 100*x_axis_rot(2), 100*x_axis_rot(3), 'r--', 'LineWidth', 2);
    quiver3(P_plane(1), P_plane(2), P_plane(3), 100*y_axis_rot(1), 100*y_axis_rot(2), 100*y_axis_rot(3), 'g--', 'LineWidth', 2);
    quiver3(P_plane(1), P_plane(2), P_plane(3), 100*z_axis_rot(1), 100*z_axis_rot(2), 100*z_axis_rot(3), 'b--', 'LineWidth', 2);

    % Plot plane normal vector (black arrow)
    quiver3(P_plane(1), P_plane(2), P_plane(3), 200*normal(1), 200*normal(2), 200*normal(3), ...
        'k', 'LineWidth', 2, 'MaxHeadSize', 2);

    zlim([0 1200]);
    legend('y=0 Plane (Rot Frame)', 'Z=0 Plane', 'Joints', ...
           'Base X', 'Base Y', 'Base Z', 'Rot X', 'Rot Y', 'Rot Z', 'Plane Normal');
    view(45, 30);  % better 3D angle
end
