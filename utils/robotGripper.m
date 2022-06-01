function [gripper, op] = robotGripper(numFingers, palmDims, segmentDims, tipRad, bendAngles, colors)
% ROBOTGRIPPER Construct a robot gripper multibody with articulated fingers.  Output op is an operating point
% that sets the initial configuration based on the provided bend angles.
%
% This is a good example of building a hierarchical multibody: the gripper has finger components,
% which are themselves multibodies.  It also demonstrates construction of an operating point with
% nontrivial hierarchy.
%
%  numFingers:  Number of radially symmetric fingers, integer
%  palmDims:    Thickness and radius of circular palm, 2-vector
%  segmentDims: Lengths and radii of finger segments, N x 2 matrix
%  tipRad:      Radius of spherical finger tips, scalar
%  bendAngles:  Angles of knuckle joints, N-vector
%  colors:      Colors of palm, segments, and tip, (N+2) x 3 matrix

%  Copyright 2021 The MathWorks, Inc.

    import simscape.Value simscape.op.* simscape.multibody.*;

    gripper = Multibody;
    addComponent(gripper, 'World', WorldFrame);

    % Build palm with connection frames for fingers
    palm = RigidBody;
    body = Solid(Cylinder(palmDims(2), palmDims(1)), UniformDensity, SimpleVisualProperties(colors(1, :)));
    addComponent(palm, 'Body', 'reference', body);
    for i = 1:numFingers
        angle = Value((i - 1) * 360 / numFingers, 'deg');
        posFrameId = ['pos' num2str(i)];
        fingFrameId = ['fing' num2str(i)];
        T = RigidTransform(StandardAxisRotation(angle, Axis.PosZ), ...
                           CylindricalTranslation(palmDims(2), angle, palmDims(1) * 0));
        addFrame(palm, posFrameId, 'reference', T);
        addFrame(palm, fingFrameId, posFrameId, ...
                 RigidTransform(AlignedAxesRotation(Axis.PosZ, Axis.NegY, Axis.PosX, Axis.PosZ)));
        addConnector(palm, fingFrameId);
    end
    addConnector(palm, 'reference');
    addComponent(gripper, 'Palm', palm);

    % Add (identical) fingers
    fing = finger(segmentDims, tipRad, colors(2:end,:));
    for i = 1:numFingers
        iStr = num2str(i);
        addComponent(gripper, ['Finger' iStr], fing);
        connect(gripper, ['Palm/fing' iStr], ['Finger' iStr '/ref']);
    end

    % Connect palm to world
    connect(gripper, 'World/W', 'Palm/reference');

    % Construct operating point for the gripper by replicating an operating
    % point for an individual finger.
    fingerOp = OperatingPoint;  % Operating point for individual finger
    for i = 1:size(segmentDims, 1)
        fingerOp("Knuckle" + num2str(i) + "/Rz/q") = Target(bendAngles(i), 'High');
    end
    op = OperatingPoint;  % Operating point for full gripper
    for i = 1:numFingers
        op("Finger" + num2str(i)) = fingerOp;
    end

end


function fing = finger(segmentDims, tipRad, colors)
% Construct an articulated gripper finger multibody comprising individual segments connected by
% revolute knuckles.  Exposes a 'ref' connector to connect to palm.
%
%  segmentDims: Lengths and radii of segments, N x 2 matrix
%  tipRad:      Radius of spherical finger tip, scalar
%  colors:      Colors of segments and tip, (N+1) x 3 matrix

    import simscape.Value simscape.multibody.*;

    numSegs = size(segmentDims, 1);  % Number of finger segments

    % Construct knuckle joint
    knuckle = RevoluteJoint;
    tsd = TorsionalSpringDamper;
    tsd.DampingCoefficient = Value(1e-3, 'N*mm/(deg/s)');
    knuckle.Rz.ForceLaws = tsd;

    fing = Multibody;

    % Add finger segments connected by knuckles
    for i = 1:numSegs
        iStr = num2str(i);
        knuckleId = ['Knuckle' iStr];
        segmentId = ['Segment' iStr];
        segment = fingerSegment(segmentDims(i, :), colors(i, :));
        addComponent(fing, knuckleId, knuckle);
        addComponent(fing, segmentId, segment);
        if i > 1
            connect(fing, [knuckleId '/B'], ['Segment' num2str(i-1) '/out']);
        end
        connect(fing, [knuckleId '/F'], [segmentId '/in']);
    end

    % Add finger tip
    tip = Solid(Sphere(tipRad), UniformDensity, SimpleVisualProperties(colors(end, :)));
    addComponent(fing, 'Tip', tip);
    connect(fing, ['Segment' num2str(numSegs) '/out'], 'Tip/R');

    % Expose connection frame at inboard side of first knuckle
    addConnector(fing, 'ref', 'Knuckle1/B');
end


function seg = fingerSegment(lenRad, color)
% Build a single gripper finger segment rigid body
%
% lenRad: Segment length and radius, 2-vector
% color:  RGB color, 3-vector

    import simscape.multibody.*

    % Build frame hierarchy
    seg = RigidBody;
    offset = lenRad(1) / 2;
    R = AlignedAxesRotation(Axis.PosZ, Axis.PosX, Axis.PosX, Axis.PosZ);
    addFrame(seg, 'in', 'reference', RigidTransform(R, StandardAxisTranslation(offset, Axis.NegZ)));
    addFrame(seg, 'out', 'reference', RigidTransform(R, StandardAxisTranslation(offset, Axis.PosZ)));

    % Add solid component
    solid = Solid(Cylinder(lenRad(2), lenRad(1)), UniformDensity, SimpleVisualProperties(color));
    addComponent(seg, 'Body', 'reference', solid);

    % Connectors
    addConnector(seg, 'in');
    addConnector(seg, 'out');
end

%  LocalWords:  fing Segs seg Rz
