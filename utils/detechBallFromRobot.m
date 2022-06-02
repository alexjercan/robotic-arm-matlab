function [robot,env] = detechBallFromRobot(robot,env,config)   
    % Get current pose of the ball attached to the hand.
    ballPose = getTransform(robot,toConfig(robot, config),"myBall");
    
    removeBody(robot,"myBall");

    ball = collisionSphere(.1);
    ball.Pose = ballPose;

    env = [{ball}, env];
end

