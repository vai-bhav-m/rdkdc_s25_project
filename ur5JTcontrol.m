function finalerr = ur5JTcontrol(gdesired, K, ur5)
% This function implements Jacobian Transpose control for a UR5 robot to reach a desired pose.
%
% Inputs:
% - gdesired: 4x4 SE(3) homogeneous transform representing the desired end-effector pose
% - K: gain scalar (try starting small, e.g., 0.1)
% - ur5: robot object with methods to read and move joints
%
% Output:
% - finalerr: final positional error (in cm) if converged, or -1 if aborted due to singularity

    Tstep = 0.1;            % Time step for control update (seconds)
    thresh_v = 0.01;        % Threshold for linear convergence (m)
    thresh_w = 1;           % Threshold for angular convergence (rad)

    max_iterations = 500;   % Safety: max number of iterations to avoid infinite loop

    iteration = 0;

    while iteration < max_iterations
        iteration = iteration + 1;

        q = ur5.get_current_joints();         % Read current joint angles
        Jb = ur5BodyJacobian(q);             % Compute body Jacobian at current config

        % Optional: check manipulability (inverse condition number)
        mu = manipulability(Jb, 'invcond');
        if mu < 0.01
            disp("WARNING: Near singularity, but continuing (Jacobian Transpose is more robust)");
        end

        gst = compute_FK_DH(q);                  % Get current end-effector pose
        Xi = getXi(pinv(gdesired) * gst);    % Compute error twist (desired vs current)

        % Check for convergence: both translational and rotational error small
        if (norm(Xi(1:3)) < thresh_v) && (norm(Xi(4:6)) < thresh_w)
            disp("REACHED: Converged to the desired configuration.");
            finalerr = norm(gst(1:3,4) - gdesired(1:3,4)) * 100;  
            return
        end

        % Jacobian Transpose Control: compute joint velocities
        qdot = K * Jb' * Xi;

        % Euler integration to get next joint configuration
        q_next = q - Tstep * qdot;
        ur5.move_joints(q_next, 5);
        pause(1);
    end

    disp("ABORTING: Maximum iterations exceeded without convergence.");
    finalerr = -1;
end
