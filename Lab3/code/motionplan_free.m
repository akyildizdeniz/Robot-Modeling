function qref = motionplan_free(q0,q2,t1,t2,myrobot,obs,tol)
    % motion planning with no obstacles
    q = q0;
    alpha = 0.001;
    while norm(q(end,1:5)-q2(1:5)) >= tol
        % Gradient descent algorithm:
        tau = alpha*att(q(end,:), q2, myrobot);
        q(end+1,:) = q(end,:) + tau;
    end
    q(:,6) = linspace(q0(6),q2(6),size(q,1));
    t = linspace(t1,t2,size(q,1));
    qref = spline(t,q'); % defines a spline object with interpolation
    % times in t and interpolation values the columns of q
end

