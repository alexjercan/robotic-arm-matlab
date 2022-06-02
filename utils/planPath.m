function [path] = planPath(robot,env,startConfig,goalConfig)
    planner = manipulatorRRT(robot,env);
    planner.MaxConnectionDistance = 0.45;
    planner.ValidationDistance = 0.1;
    planner.EnableConnectHeuristic = false;
    
    plannedPath = plan(planner, startConfig, goalConfig);
    shortenedPath = shorten(planner, plannedPath, 20);
    path = interpolate(planner, shortenedPath, 20);
end

