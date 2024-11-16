function updated_W_xu_ranges = updateWxuRanges(estimated_W_xu, range_factor)
    % estimated_W_x is the previous estimate
    % range_factor controls how wide the new range should be
    updated_W_xu_ranges = zeros(size(estimated_W_xu, 1), 2);
    
    for i = 1:length(estimated_W_xu)
        % Define new range centered around the estimated weight
        center = estimated_W_xu(i);
        range = center * range_factor;
        updated_W_xu_ranges(i, :) = [center - range, center + range];
    end
end