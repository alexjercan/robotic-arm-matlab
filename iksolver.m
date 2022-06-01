[ur5e_robot,config, partFeeder, toolStationBase, platform, box0, box1] = create();
env = {partFeeder, toolStationBase, platform, box0, box1};

weights = [1 1 1 1 1 1];
initialguess = ur5e_robot.homeConfiguration;
ikPoints = zeros(3, size(initialguess, 2));
ik = generalizedInverseKinematics('RigidBodyTree', ur5e_robot, 'ConstraintInputs', {'cartesian', 'pose'});
heightAboveTable = constraintCartesianBounds('forearm_link');
heightAboveTable.Bounds = [-inf, inf; ...
                           -inf, inf; ...
                           0.3, inf];
poseToTarget = constraintPoseTarget('tool0');

m = trvec2tform([0 0 0.2]) * eul2tform([0, pi, 0]);
poseToTarget.TargetTransform = partFeeder.Pose * m;
initialguess = homeConfiguration(ur5e_robot);
[configSoln, ~] = ik(initialguess,heightAboveTable,poseToTarget);
configSolnA = fromConfig(configSoln);
ikPoints(1, :) = configSolnA;

m = trvec2tform([0 0 0.36]) * eul2tform([0, pi, 0]);
poseToTarget.TargetTransform = box0.Pose * m;
initialguess = homeConfiguration(ur5e_robot);
[configSoln, ~] = ik(initialguess,heightAboveTable,poseToTarget);
configSolnA = fromConfig(configSoln);
ikPoints(2, :) = configSolnA;

m = trvec2tform([0 0 0.36]) * eul2tform([0, pi, 0]);
poseToTarget.TargetTransform = box1.Pose * m;
initialguess = homeConfiguration(ur5e_robot);
[configSoln, ~] = ik(initialguess,heightAboveTable,poseToTarget);
configSolnA = fromConfig(configSoln);
ikPoints(3, :) = configSolnA;
