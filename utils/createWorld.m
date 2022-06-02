function [ur5e_robot,env,ikPoints] = createWorld()
    [ur5e_robot, ~, partFeeder, toolStationBase, platform, box0, box1] = createModels();
    ikPoints = myIKSolver(ur5e_robot, partFeeder, box0, box1);
    env = {partFeeder, toolStationBase, platform, box0, box1};
end

