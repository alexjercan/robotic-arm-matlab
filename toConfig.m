function [outputConfig] = toConfig(robot, config)
    outputConfig = homeConfiguration(robot);

    for i = 1:numel(config)
        outputConfig(i).JointPosition = config(i);
    end
end

