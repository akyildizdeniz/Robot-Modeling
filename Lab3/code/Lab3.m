%% Definition of robot structure
% DH Table 
% All units in cm.
DH = [0, 76, 0, pi/2; ...
      0, -23.65, 43.24, 0; ...
      0, 0, 0, pi/2; ...
      0, 43.18, 0, -pi/2; ...
      0, 0, 0, pi/2; ...
      0, 20, 0, 0];
% Create the robot model.
myrobot = mypuma560(DH);

%% 3.1 The Attractive Field

% Testing the attractive torque values
H1 = eul2tr([0 pi pi/2]); % eul2tr converts ZYZ Euler angles to a hom. tsf. mtx
H1(1:3,4)=100*[-1; 3; 3;]/4; % This assigns the desired displacement to the hom.tsf.mtx.
q1 = inverse(H1,myrobot);
% This is the starting joint variable vector.
H2 = eul2tr([0 pi -pi/2]);
H2(1:3,4)=100*[3; -1; 2;]/4;
q2 = inverse(H2,myrobot);
% This is the final joint variable vector
% Verify the function works:
disp('The resulting attractive component:')
tau = att(q1,q2,myrobot)

%% 3.2 Motion Planning without Obstacles

% Using the motion planning without obstacles
% It moves a little bit slowly, but reaches the goal.
qref = motionplan_free(q1,q2,0,10,myrobot,[],0.01);
t=linspace(0,10,300);
q = ppval(qref,t)';
plot(myrobot,q)

%% 3.3 Motion Planning with Obstacles

% First, verify the repulsive torque values
setupobstacle
q3 = 0.9*q1+0.1*q2;
disp('The resulting repulsive component for obstacle 1:')
tau = rep(q3,myrobot,obs{1})
q = [pi/2 pi 1.2*pi 0 0 0];
disp('The resulting repulsive component for obstacle 6:')
tau = rep(q,myrobot,obs{6})

% With the current configuration, this part takes a bit long to run
% since it computes for all the obstacles in a scene, and then seems 
% to get stuck in between obstacles, however it eventually
% reaches the desired position as requested in the lab handout.
hold on
axis([-100 100 -100 100 0 200])
view(-32,50)
plotobstacle(obs);
qref = motionplan(q1,q2,0,10,myrobot,obs,0.01);
t=linspace(0,10,300);
q=ppval(qref,t)';
plot(myrobot,q);
hold off

