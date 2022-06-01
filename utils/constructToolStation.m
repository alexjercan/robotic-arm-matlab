function [base, feeder] = constructToolStation(globalPose, showResults)

    if nargin < 2
        showResults = false;
    end

    % Create the base
    base = collisionBox(.1, .1, .2);
    base.Pose = trvec2tform([0 0 base.Z/2]);

    % Add the part feeder and position relative to base
    feeder = collisionSphere(.1);
    feeder.Pose = trvec2tform([0 0 base.Pose(3,4)+base.Z/2+feeder.Radius]);

    % Position all the pieces globally
    base.Pose = globalPose*base.Pose;
    feeder.Pose = globalPose*feeder.Pose;

    % Show results
    if showResults
        figure();
        ax = gca;
        base.show('Parent', ax);
        hold all
        feeder.show('Parent', ax);
    end
end