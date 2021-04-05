function myrobot = mypuma560(DH)
    % Function to create a robot model from given DH table.
    L(1) = Link([DH(1,:)], 'standard');
    L(2) = Link([DH(2,:)], 'standard');
    L(3) = Link([DH(3,:)], 'standard');
    L(4) = Link([DH(4,:)], 'standard');
    L(5) = Link([DH(5,:)], 'standard');
    L(6) = Link([DH(6,:)], 'standard');
    myrobot = SerialLink(L, 'name', 'myrobot');
end