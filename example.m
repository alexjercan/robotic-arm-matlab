% Setup
clear
clc
addpath(genpath(pwd));

% Create an image of the ball
circlePixels = mkCircle();

% Display the image
image(circlePixels);
title('Binary image of a circle');
pause(1);

% Based on the color of the ball choose target to be 2 or 3 TODO
targetBoxIndex = 2;

% Set the seed for the movement of the robot arm
rng(42);

% Create env and compute ik waypoints
tic;
[ur5e_robot,env,ikPoints] = createWorld();
elapsedTime = toc;
disp(['Created world in ',num2str(elapsedTime), ' seconds']);

% Waypoints to use in sim
startConfig = ikPoints(1,:);
goalConfig = ikPoints(targetBoxIndex,:);

% Attach the ball to the arm
[ur5e_robot,env] = attachBallToRobot(ur5e_robot,env,startConfig);

% Path planning
tic;
path = planPath(ur5e_robot,env,startConfig,goalConfig);
elapsedTime = toc;
disp(['Done planning for segment in ',num2str(elapsedTime), ' seconds']) %i in %f seconds\n])

% Show path taken
clf();
showPath(ur5e_robot,env,path);

% Detach ball from the arm
[ur5e_robot,env] = detechBallFromRobot(ur5e_robot,env,goalConfig);

% Waypoints to use in sim
startConfig = ikPoints(targetBoxIndex,:);
goalConfig = ikPoints(1,:);

% Path planning
tic;
path = planPath(ur5e_robot,env,startConfig,goalConfig);
elapsedTime = toc;
disp(['Done planning for segment in ',num2str(elapsedTime), ' seconds']) %i in %f seconds\n])

% Show path taken
clf();
showPath(ur5e_robot,env,path);