function compute_errors(p_sources, group_best_positions, max_error)
    n_sources = size(p_sources, 1); % Number of true sources
    error_distances = zeros(1, n_sources); % Store the error distances for valid estimates
    
    % Iterate through each true source
    for i = 1:n_sources
        source = p_sources(i, :);  % Get the true source position
        
        % Find the nearest estimate from the group best positions
        distances = sqrt(sum((group_best_positions - source).^2, 2));  % Euclidean distance
        
        % Filter out estimates that exceed the max error threshold
        valid_estimates = distances <= max_error;
        
        % If there are valid estimates, take the one with the smallest error
        if any(valid_estimates)
            [min_distance, ~] = min(distances(valid_estimates));
            error_distances(i) = min_distance;
            fprintf('Source %d: Nearest estimate is %.2f meters away (within max error)\n', i, min_distance);
        else
            fprintf('Source %d: No valid estimate within %.2f meters\n', i, max_error);
        end
    end
    
    % Calculate overall error for the valid estimates
    total_error = sum(error_distances)/n_sources;
    fprintf('Total error for all valid estimates: %.2f meters\n', total_error);
end
