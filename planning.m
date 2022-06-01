load waypointData.mat

rng(10);
[ur5e_robot,config, partFeeder, toolStationBase, platform, box0, box1] = create();
env = {partFeeder, toolStationBase, platform, box0, box1};

planner = manipulatorRRT(ur5e_robot,env);
planner.MaxConnectionDistance = 0.45;
planner.ValidationDistance = 0.1;
planner.EnableConnectHeuristic = false;

indices = [1 2; 1 3; 2 1; 3 1;];
numWaypoints = size(indices, 1);

paths = cell(1,numWaypoints);
for segIdx = 1:numWaypoints
    tic;
    startIdx = indices(segIdx, 1);
    endIdx = indices(segIdx, 2);

    plannedPath = plan(planner, waypointData(startIdx,:), waypointData(endIdx,:));
    shortenedPath = shorten(planner, plannedPath, 20);
    paths{segIdx} = interpolate(planner, shortenedPath, 20);

    segTime = toc;
    disp(['Done planning for segment ',num2str(segIdx),' in ',num2str(segTime), ' seconds']) %i in %f seconds\n])
end