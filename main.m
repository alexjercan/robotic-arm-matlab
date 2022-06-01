load waypointData.mat
env = {partFeeder toolStationBase toolCheckingBase platform box0 box1};
planner = manipulatorRRT(ur5e_robot,env);
planner.MaxConnectionDistance = 0.45;
planner.ValidationDistance = 0.1;

% rng(10);
numPts = 25;
numWaypoints = size(waypointData,1);
paths = cell(1,numWaypoints);
for segIdx = 1:numWaypoints
    tic;
    plannedPath = plan(planner, waypointData(segIdx,:), waypointData(mod(segIdx,numWaypoints)+1,:));
    shortenedPath = shorten(planner, plannedPath, 10);
    paths{segIdx} = interpolate(planner, shortenedPath, 10);

    segTime = toc;
    disp(['Done planning for segment ',num2str(segIdx),' in ',num2str(segTime), ' seconds']) %i in %f seconds\n])
end

totalSegs = vertcat(paths{:});