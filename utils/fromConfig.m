function [outputConfig] = fromConfig(config)
    outputConfig = zeros(1, size(config, 2));

    for i = 1:numel(config)
        outputConfig(i) = config(i).JointPosition;
    end
end

