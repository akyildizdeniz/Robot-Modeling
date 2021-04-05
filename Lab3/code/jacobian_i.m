function J = jacobian_i(joint, myrobot, k)
    % Function to compute the jacobian(o_i),
    % given the robot structure and joint variables
    H_all = forward_jacobian(joint, myrobot);
    J_v = zeros(3,6);
    % Using the equation 2 provided in the handout
    for i = 1:6
        if i == 1
            o = zeros(3,1);
            z = [0;0;1];
        else
            o = H_all(1:3,4,i-1);
            z = H_all(1:3,3,i-1);
        end
        Jv_i = cross(z, (H_all(1:3,4,k) - o));
        J_v(:,i) = Jv_i;
    end
    J = J_v;
end

