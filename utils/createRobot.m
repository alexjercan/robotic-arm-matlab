function [robot] = createRobot()
    robot = loadrobot("universalUR5e");

    handBase = rigidBody("handBase");
    finger1 = rigidBody("finger1");
    finger2 = rigidBody("finger2");
    finger3 = rigidBody("finger3");
    finger4 = rigidBody("finger4");

    handBaseDims = [0.05,0.01];
    fingerDims = [0.01,0.01];
    epsT = trvec2tform([0 0 1e-2]);

    collHandBase = collisionCylinder(handBaseDims(1),handBaseDims(2)/2);
    collHandBase.Pose = trvec2tform([0 0 handBaseDims(2)]) * epsT;

    epsT = trvec2tform([0 0 2e-2]);
    collFinger1 = collisionCylinder(fingerDims(1),fingerDims(2));
    collFinger1.Pose = trvec2tform([0 handBaseDims(1) handBaseDims(2)]) * trvec2tform([0 -fingerDims(1) fingerDims(2)/2]) * epsT;

    collFinger2 = collisionCylinder(fingerDims(1),fingerDims(2));
    collFinger2.Pose = trvec2tform([handBaseDims(1) 0 handBaseDims(2)]) * trvec2tform([-fingerDims(1) 0 fingerDims(2)/2]) * epsT;

    collFinger3 = collisionCylinder(fingerDims(1),fingerDims(2));
    collFinger3.Pose = trvec2tform([0 -handBaseDims(1) handBaseDims(2)]) * trvec2tform([0 fingerDims(1) fingerDims(2)/2]) * epsT;

    collFinger4 = collisionCylinder(fingerDims(1),fingerDims(2));
    collFinger4.Pose = trvec2tform([-handBaseDims(1) 0 handBaseDims(2)]) * trvec2tform([fingerDims(1) 0 fingerDims(2)/2]) * epsT;

    handBaseJoint = rigidBodyJoint("handBaseJoint");
    finger1Joint = rigidBodyJoint("finger1Joint");
    finger2Joint = rigidBodyJoint("finger2Joint");
    finger3Joint = rigidBodyJoint("finger3Joint");
    finger4Joint = rigidBodyJoint("finger4Joint");

    handBase.Joint = handBaseJoint;
    finger1.Joint = finger1Joint;
    finger2.Joint = finger2Joint;
    finger3.Joint = finger3Joint;
    finger4.Joint = finger4Joint;

    addCollision(handBase,collHandBase);
    addCollision(finger1,collFinger1);
    addCollision(finger2,collFinger2);
    addCollision(finger3,collFinger3);
    addCollision(finger4,collFinger4);

    addBody(robot, handBase, "tool0");
    addBody(robot, finger1, "handBase");
    addBody(robot, finger2, "handBase");
    addBody(robot, finger3, "handBase");
    addBody(robot, finger4, "handBase");
end

