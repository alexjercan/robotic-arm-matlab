function [gripper,gripper_op] = myRobotGripper()   
    import simscape.Value;
    
    numFingers = 5;
    
    % Palm dimensions
    palmDims = Value([20 100], 'mm');
    
    % Finger segment number and dimensions
    segmentDims = Value([80 10; ...   % Proximal segment
                        70 10; ...    % Middle segment
                        70 10], ...   % Distal segment
                        'mm');
    
    % Finger tip dimensions
    tipRad = Value(20, 'mm');
    
    % Finger bend angles
    bendAngles = Value([-30, +30, +25], 'deg');
    
    % Finger segment colors
    colors = [0 0 .7;  ...     % Palm
              .5 .4 0;  ...    % Proximal segment
              .8 .6 0; ...     % Middle segment
              1 .9 0; ...      % Distal segment
              1 1 1];          % Tip
    
    % Construct the gripper
    [gripper, gripper_op] = robotGripper(numFingers, palmDims, segmentDims, tipRad, bendAngles, colors);
end

