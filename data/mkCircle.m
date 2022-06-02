function [pixels] = mkCircle()
    imageSizeX = 640;
    imageSizeY = 480;
    [columnsInImage,rowsInImage] = meshgrid(1:imageSizeX, 1:imageSizeY);
    
    centerX = 320;
    centerY = 240;
    radius = 100;
    circlePixels = (rowsInImage - centerY).^2 ...
        + (columnsInImage - centerX).^2 <= radius.^2;
    
    pixels = ones(imageSizeY, imageSizeX, 3);
    
    number = rand(1);
    if number < 0.5
        pixels(:, :, 1) = pixels(:, :, 1) - circlePixels;
    end
    pixels(:, :, 2) = pixels(:, :, 2) - circlePixels;
    pixels(:, :, 3) = pixels(:, :, 3) - circlePixels;
end

