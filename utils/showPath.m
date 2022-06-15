function [points] = showPath(robot,env,path,name)
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

    points = zeros(size(path,1), 3);
    for configIdx = 1:size(path,1)
        tool0T = getTransform(showRobot, path(configIdx,:), 'tool0');
        point = tool0T * [0;0;0;1];
        points(configIdx, :) = point(1:3);
    end

    % For each path segment, step through all the configurations
    plot3(points(:,1), points(:,2), points(:,3));
    for configIdx = 1:size(path,1)
        show(showRobot, path(configIdx,:), 'Collisions','on','Visuals','on', "FastUpdate",true, "PreservePlot",false,"Parent",ax);

        exportgraphics(gcf,sprintf('out/%s_%05d.png',name,configIdx),'Resolution',300)
        waitfor(r);
    end

    % Hold the pose and update the title each time a waypoint is reached
    title(sprintf('Segment completed'), "Parent",ax);
    hold off
end

