function [ur5e_robot,env,ikPoints] = createWorld()
    % Load robot
    ur5e_robot = loadrobot("universalUR5e");
    
    % Create gripper
    [gripper,gripper_op] = myRobotGripper();
    if isempty(which('gripperModel'))
        makeBlockDiagram(gripper,gripper_op,'gripperModel');
    end
    % Add gripper to robot
    [gripper, ~] = importrobot('gripperModel');
    addSubtree(ur5e_robot,'tool0',gripper);
    
    % build environment
    toolStationBase = collisionBox(.2, .2, .4);
    toolStationBase.Pose = trvec2tform([0 0.4 toolStationBase.Z/2]);

    ballSphere = collisionSphere(.1);
    ballSphere.Pose = trvec2tform([0 0.4 toolStationBase.Pose(3,4)+toolStationBase.Z/2+ballSphere.Radius]);
    
    platform = collisionBox(1,1,0.02);
    platform.Pose = trvec2tform([0 0 -.011]);

    box0 = collisionBox(0.2,0.3,0.1);
    box0.Pose = trvec2tform([0.3 -0.3 .05]);
    
    box1 = collisionBox(0.2,0.3,0.1);
    box1.Pose = trvec2tform([-0.3 -0.3 .05]);

    jointConfig = homeConfiguration(ur5e_robot);
    numJoints = size(jointConfig, 2);
    config = zeros(1, numJoints);
    for i = 1:numJoints
        config(1, i) = jointConfig(i).JointPosition;
    end

    ikPoints = myIKSolver(ur5e_robot, ballSphere, box0, box1);
    env = {ballSphere, toolStationBase, platform, box0, box1};
end

