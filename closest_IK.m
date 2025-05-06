function best_theta = closest_IK(g, q)
    % Returns IK solution closest to current joint state q
    thetas = ur5InvKin(g);
    [~, idx] = min(vecnorm(thetas - q));
    best_theta = thetas(:, idx);
end