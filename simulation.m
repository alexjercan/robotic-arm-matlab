ax = show(ur5e_robot);
hold all
for i = 1:numel(env)
    env{i}.show("Parent", ax);
end

% Display the figure window the animation
pathFig = ancestor(ax, 'figure');
set(pathFig, "Visible", "on")

% Set up timing and configure robot
r = rateControl(10);
tvec = linspace(1,numWaypoints,numWaypoints*numPts);
ur5e_robot.DataFormat = "row";

% Animate all path segments
for pathSegIdx = 1:numel(paths)
    path = paths{pathSegIdx};

    % For each path segment, step through all the configurations
    for configIdx = 1:size(path,1)
        show(ur5e_robot, path(configIdx,:), "FastUpdate",true, "PreservePlot",false,"Parent",ax);
        waitfor(r);
    end

    % Hold the pose and update the title each time a waypoint is reached
    title(sprintf('Segment %i completed', pathSegIdx), "Parent",ax);
    pause(1);
end