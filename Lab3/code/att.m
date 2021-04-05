function tau = att(q,q2,myrobot)
    % Function to compute attractive torques
    % Solve FK for qs and qf
    H_start = forward_jacobian(q, myrobot);
    H_final = forward_jacobian(q2, myrobot);
    % z value could be adjusted here
    z = 1;
    % Using the textbook equation 5.4 here:
    tau = zeros(1,6);
    for i = 1:6
        o_start(:,i) = H_start(1:3,4,i);
        o_final(:,i) = H_final(1:3,4,i);
        F_att(:,i) = -z * (o_start(:,i) - o_final(:,i));
        J = jacobian_i(q, myrobot,i);
        J_o_i = horzcat(J(1:3, 1:i),zeros(3,6-i));
        tau = tau + (J_o_i' * F_att(:,i))';
    end
    if norm(tau) ~= 0
        tau = tau / norm(tau);
    end
end

