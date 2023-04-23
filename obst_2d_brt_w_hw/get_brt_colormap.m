function [colormap] = get_brt_colormap(brt_value,theta)
% return colormap based on current theta value

% get theta index
    theta_index = ceil ((-1*theta + pi)/(2*pi) * 20);

    brt_value = brt_value(:,:,:,end); % get brt value at 2s timestep (final)
    map_t = brt_value(:,:,theta_index);
    map_t(map_t > 0) = 0;
    map_t = mat2gray(-1*map_t);
    map_t = im2uint8(map_t);

    targetSize = [864 480];
    colormap = imresize(map_t,targetSize)';

end