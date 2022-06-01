function [ur5e_robot,env] = create()
    % Load robot
    ur5e_robot = loadrobot("universalUR5e");
    
    % % Create gripper
    % [gripper,gripper_op] = myRobotGripper();
    % if isempty(which('gripperModel'))
    %     makeBlockDiagram(gripper,gripper_op,'gripperModel');
    % end
    %
    % % Add gripper to robot
    % [gripper, importInfo] = importrobot('gripperModel');
    % addSubtree(ur5e_robot,'tool0',gripper);
    
    % build environment
    stationPose = trvec2tform([0 0.4 0]);
    [toolStationBase,partFeeder] = constructToolStation(stationPose);
    
    platform = collisionBox(1,1,0.02);
    platform.Pose = trvec2tform([0 0 -.011]);
    
    box0 = collisionBox(0.2,0.3,0.1);
    box0.Pose = trvec2tform([0.3 -0.3 .05]);
    
    box1 = collisionBox(0.2,0.3,0.1);
    box1.Pose = trvec2tform([-0.3 -0.3 .05]);

    env = {partFeeder toolStationBase platform box0 box1};
end

