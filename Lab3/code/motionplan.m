function qref = motionplan(q0,q2,t1,t2,myrobot,obs,tol)
    % Motion planning with multiple obstacles
    q = q0;
    alpha = 0.01;
    while norm(q(end,1:5)-q2(1:5)) >= tol
        % Gradient descent algorithm:
        tau_rep = zeros(1,6);
        for k=1:size(obs,2)
            tau_rep = tau_rep + alpha*rep(q(end,:),myrobot,obs{k});
        end
        % Add the sum of repulsive torques 
        tau = alpha*att(q(end,:), q2, myrobot) + tau_rep;
        q(end+1,:) = q(end,:) + tau;
    end
    q(:,6) = linspace(q0(6),q2(6),size(q,1));
    t = linspace(t1,t2,size(q,1));
    qref = spline(t,q'); % defines a spline object with interpolation
    % times in t and interpolation values the columns of q
end