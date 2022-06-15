function [pixels] = mkCircle(circle)
    imageSizeX = 640;
    imageSizeY = 480;
    [columnsInImage,rowsInImage] = meshgrid(1:imageSizeX, 1:imageSizeY);
    
    centerX = 320;
    centerY = 240;
    radius = 100;
    circlePixels = (rowsInImage - centerY).^2 ...
        + (columnsInImage - centerX).^2 <= radius.^2;
    
    pixels = ones(imageSizeY, imageSizeX, 3);
    
    if circle == 0
        number = rand(1);
    elseif circle == 1
        number = 0;
    else
        number = 1;
    end

    if number < 0.5
        pixels(:, :, 1) = pixels(:, :, 1) - circlePixels;
    end
    pixels(:, :, 2) = pixels(:, :, 2) - circlePixels;
    pixels(:, :, 3) = pixels(:, :, 3) - circlePixels;
end

