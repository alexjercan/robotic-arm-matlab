% Setup
clear
clc
addpath(genpath(pwd));

[ur5e_robot,env,ikPoints] = createWorld(1);

startConfig = toConfig(ur5e_robot, ikPoints(1,:));

ax = show(ur5e_robot, startConfig, 'Collisions','on');
