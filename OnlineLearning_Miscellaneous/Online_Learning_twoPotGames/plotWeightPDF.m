function plotWeightPDF(weights)
    % Plot histogram
    figure;
    histogram(weights, 'Normalization', 'pdf');
%     histogram(pre_resample_weights, 'Normalization', 'pdf', 'BinWidth', 0.01); % Adjust bin width
    title('PDF of Resampled Weights');
    xlabel('Weight');
    ylabel('Probability Density');

    % Plot Kernel Density Estimation (KDE) for smoother PDF
    [f, xi] = ksdensity(weights);
%     [f_pre, xi_pre] = ksdensity(pre_resample_weights, 'Bandwidth', 0.1); % Adjust bandwidth

    hold on;
    plot(xi, f, 'LineWidth', 2);
    legend('Histogram', 'KDE');
    hold off;
end