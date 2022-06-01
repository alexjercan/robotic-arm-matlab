load waypointData.mat

% Create env
[ur5e_robot,config, partFeeder, toolStationBase, platform, box0, box1] = create();
env = {partFeeder, toolStationBase, platform, box0, box1};

% Waypoints to use in sim
startConfig = waypointData(1,:);
goalConfig = waypointData(2,:);

% Create can as a rigid body
cylinder1 = env{1};
canBody = rigidBody("myCan");
canJoint = rigidBodyJoint("canJoint");

% Get current pose of the robot hand.
endEffectorPose = getTransform(ur5e_robot,toConfig(ur5e_robot, startConfig),"tool0");

% Place can into the end effector gripper.
setFixedTransform(canJoint,endEffectorPose\cylinder1.Pose); 

% Add collision geometry to rigid body.
addCollision(canBody,cylinder1,inv(cylinder1.Pose));
canBody.Joint = canJoint;

% Add rigid body to robot model.
addBody(ur5e_robot,canBody,"tool0");

% Remove object from env
env(1) = [];

% Path planning
tic;
planner = manipulatorRRT(ur5e_robot,env);
planner.MaxConnectionDistance = 0.45;
planner.ValidationDistance = 0.1;
planner.EnableConnectHeuristic = false;

plannedPath = plan(planner, startConfig, goalConfig);
shortenedPath = shorten(planner, plannedPath, 20);
path = interpolate(planner, shortenedPath, 20);

segTime = toc;
disp(['Done planning for segment 1 in ',num2str(segTime), ' seconds']) %i in %f seconds\n])

ax = show(ur5e_robot);
hold all
for i = 1:numel(env)
    env{i}.show("Parent", ax);
end

% Display the figure window the animation
pathFig = ancestor(ax, 'figure');
set(pathFig, "Visible", "on")

% Set up timing and configure robot
r = rateControl(10);
ur5e_robot.DataFormat = "row";

% For each path segment, step through all the configurations
for configIdx = 1:size(path,1)
    show(ur5e_robot, path(configIdx,:), 'Collisions','on','Visuals','off', "FastUpdate",true, "PreservePlot",false,"Parent",ax);
    waitfor(r);
end

% Hold the pose and update the title each time a waypoint is reached
title(sprintf('Segment 1 completed'), "Parent",ax);
pause(1);

