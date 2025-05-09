
function finalerr = ur5JTcontrol(gdesired, K, ur5)
    % UR5 Jacobian Transpose control for end-effector pose tracking.
    % Inputs:
    %   gdesired - Desired end-effector pose (4x4 homogeneous transform)
    %   K        - Scalar control gain
    %   ur5      - UR5 robot object with joint interface
    % Output:
    %   finalerr - [SO(3) rotational error, R³ positional error] if converged,
    %              -1 if failed to converge

    % Control parameters
    Tstep = 0.25/K;                   % Fixed time step (seconds)
    thresh_v = 0.005;               % Linear velocity threshold (m)
    thresh_w = 1;                  % Angular velocity threshold (rad)
    max_iterations = 20;           % Maximum number of iterations

    iteration = 0;

    while iteration < max_iterations
        iteration = iteration + 1;

        % Get current joint angles and Jacobian
        q = ur5.get_current_joints();
        Jb = ur5BodyJacobian(q);
        mu = manipulability(Jb, 'invcond');

        if mu < 0.01
            disp("WARNING: Near singularity, but continuing (Jacobian Transpose is robust)");
        end

        % Compute current end-effector pose and twist error
        gst = compute_FK_DH(q);  % Use your external DH-based FK function
        Xi = getXi(pinv(gdesired) * gst);  % Correct error computation
        v_k = Xi(1:3);
        omega_k = Xi(4:6);

        % Check convergence
        if norm(v_k) < thresh_v && norm(omega_k) < thresh_w
            disp("REACHED: Converged to the desired configuration.");

            % Compute final pose error
            R_err = gst(1:3,1:3) - gdesired(1:3,1:3);
            d_SO3 = sqrt(trace(R_err' * R_err));
            d_R3 = norm(gst(1:3,4) - gdesired(1:3,4));
            finalerr = [d_SO3, d_R3];
            return;
        end

        % Compute next joint configuration using Jacobian transpose
        q_next = q - K * Tstep * Jb.' * Xi;
        %% 

        % Wrap joint angles to [-pi, pi]
        q_next = wrapToPi(q_next);

        % Safety check (if implemented in your system)
        if exist('safety', 'file') && ~safety(q_next, gdesired(3,4) - 0.02)
            disp("ABORTING: Safety constraint triggered.");
            finalerr = -1;
            return;
        end
        
        iteration
        % Send joint command to robot
        ur5.move_joints(q_next, 3);
        pause(4);
    end

    % If maximum iterations exceeded
    % disp("ABORTING: Maximum iterations exceeded without convergence.");
    finalerr = -1;
end

function q_wrapped = wrapToPi(q)
    % Wrap joint angles to [-pi, pi]
    q_wrapped = mod(q + pi, 2*pi) - pi;
end