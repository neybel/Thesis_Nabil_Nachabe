function updated_W_x_ranges = updateWxRanges(estimated_W_x, range_factor)
    % estimated_W_x is the previous estimate
    % range_factor controls how wide the new range should be
    updated_W_x_ranges = zeros(size(estimated_W_x, 1), 2);
    
    for i = 1:length(estimated_W_x)
        % Define new range centered around the estimated weight
        center = estimated_W_x(i);
        range = center * range_factor;
        updated_W_x_ranges(i, :) = [center - range, center + range];
    end
end