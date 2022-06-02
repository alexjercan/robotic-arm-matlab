function showPath(robot,env,path)
    showRobot = copy(robot);

    % Display the world
    ax = show(showRobot);
    hold all
    for i = 1:numel(env)
        env{i}.show("Parent", ax);
    end

    % Display the figure window the animation
    pathFig = ancestor(ax, 'figure');
    set(pathFig, "Visible", "on")
    
    % Set up timing and configure robot
    r = rateControl(10);
    showRobot.DataFormat = "row";
    
    % For each path segment, step through all the configurations
    for configIdx = 1:size(path,1)
        show(showRobot, path(configIdx,:), 'Collisions','on','Visuals','on', "FastUpdate",true, "PreservePlot",false,"Parent",ax);
        waitfor(r);
    end
    
    % Hold the pose and update the title each time a waypoint is reached
    title(sprintf('Segment completed'), "Parent",ax);
end

