function indices = systematicResample(weights)
    N = length(weights);
    positions = (0:N-1) / N + rand / N;
    indices = zeros(1, N);
    cumulative_sum = cumsum(weights);
    i = 1;
    j = 1;
    while i <= N
        if positions(i) < cumulative_sum(j)
            indices(i) = j;
            i = i + 1;
        else
            j = j + 1;
        end
    end
end