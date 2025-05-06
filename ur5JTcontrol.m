function finalerr = ur5JTcontrol(gdesired, K, ur5)
Tstep = 1/K; % Time step for control update (seconds)
thresh_v = 0.01; % Threshold for linear convergence (m)
thresh_w = 1; % Threshold for angular convergence (rad)
max_iterations = 500; % Safety limit
iteration = 0;
while iteration < max_iterations
iteration = iteration + 1;
q = ur5.get_current_joints();
Jb = ur5BodyJacobian(q);
mu = manipulability(Jb, 'invcond');
if mu < 0.01
disp("WARNING: Near singularity, but continuing (Jacobian Transpose is robust)");
end
gst = compute_FK_DH(q);
Xi = getXi(inv(gdesired) * gst);
v_k = Xi(1:3);
omega_k = Xi(4:6);
% Check for convergence
if (norm(v_k) < thresh_v) && (norm(omega_k) < thresh_w)
disp("REACHED: Converged to the desired configuration.");
% Compute final error (SO(3) + R³)
d_SO3 = sqrt(trace((gst(1:3,1:3) - gdesired(1:3,1:3))' * (gst(1:3,1:3) - gdesired(1:3,1:3))));
d_R3 = norm(gst(1:3,4) - gdesired(1:3,4));
finalerr = [d_SO3, d_R3];
return
end
qdot = K * Jb' * Xi;
q_next = q - Tstep * qdot;
ur5.move_joints(q_next, 5);
pause(1);
end
disp("ABORTING: Maximum iterations exceeded without convergence.");
finalerr = -1;
end