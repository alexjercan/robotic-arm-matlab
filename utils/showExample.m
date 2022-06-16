function showExample(world,circle,save)
    % Create an image of the ball
    circlePixels = mkCircle(circle);
    
    % Display the image
    image(circlePixels);
    title('Binary image of a circle');
    pause(1);
    
    % Based on the color of the ball choose target to be 2 or 3
    centerX = 320;
    centerY = 240;
    if (circlePixels(centerY,centerX,1) == 0) && (circlePixels(centerY,centerX,2) == 0) && (circlePixels(centerY,centerX,3) == 0)
        targetBoxIndex = 2;
    else
        targetBoxIndex = 3;
    end
    
    % Set the seed for the movement of the robot arm
    rng(42);
    
    % Create env and compute ik waypoints
    tic;
    [ur5e_robot,env,ikPoints] = createWorld(world);
    elapsedTime = toc;
    disp(['Created world in ',num2str(elapsedTime), ' seconds']);
    
    % Waypoints to use in sim
    startConfig = ikPoints(1,:);
    goalConfig = ikPoints(targetBoxIndex,:);
    
    % Attach the ball to the arm
    [ur5e_robot,env] = attachBallToRobot(ur5e_robot,env,startConfig);
    
    % Path planning
    tic;
    fileName = "path" + int2str(world) + "_1_" + int2str(targetBoxIndex) + ".mat";
    if isfile(fileName)
        path = load(fileName).path;
    else
        path = planPath(ur5e_robot,env,startConfig,goalConfig);
        save(fileName,"path");
    end
    elapsedTime = toc;
    disp(['Done planning for segment in ',num2str(elapsedTime), ' seconds']) %i in %f seconds\n])
    
    % Show path taken
    clf();
    fileName = "world" + int2str(world) + "_" + "circle" + int2str(circle) + "_1";
    showPath(ur5e_robot,env,path, fileName, save);
    
    % Detach ball from the arm
    [ur5e_robot,env] = detechBallFromRobot(ur5e_robot,env,goalConfig);
    
    % Waypoints to use in sim
    startConfig = ikPoints(targetBoxIndex,:);
    goalConfig = ikPoints(1,:);
    
    % Path planning
    tic;
    fileName = "path" + int2str(world) + "_" + int2str(targetBoxIndex) + "_1"  + ".mat";
    if isfile(fileName)
        path = load(fileName).path;
    else
        path = planPath(ur5e_robot,env,startConfig,goalConfig);
        save(fileName,"path");
    end
    elapsedTime = toc;
    disp(['Done planning for segment in ',num2str(elapsedTime), ' seconds']) %i in %f seconds\n])
    
    % Show path taken
    clf();
    fileName = "world" + int2str(world) + "_" + "circle" + int2str(circle) + "_2";
    showPath(ur5e_robot,env,path, fileName, save);
end

