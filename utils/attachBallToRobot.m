function [robot,env] = attachBallToRobot(robot,env,config)
    % Create can as a rigid body
    ball = env{1};
    ballBody = rigidBody("myBall");
    ballJoint = rigidBodyJoint("ballJoint");
    
    % Get current pose of the robot hand.
    endEffectorPose = getTransform(robot,toConfig(robot, config),"tool0");
    
    % Place can into the end effector gripper.
    setFixedTransform(ballJoint,endEffectorPose\ball.Pose); 
    
    % Add collision geometry to rigid body.
    addCollision(ballBody,ball,inv(ball.Pose));
    ballBody.Joint = ballJoint;
    
    % Add rigid body to robot model.
    addBody(robot,ballBody,"tool0");
    
    % Remove object from env
    env(1) = [];
end

