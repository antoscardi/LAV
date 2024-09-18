clear; close all; clc;

% Parameters
N_drones = 2;           % Number of drones (particles)
N_iterations = 500;      % Number of iterations
bounds = [-100, 100];    % Search space boundaries
max_velocity = 1.5;      % Maximum allowable velocity (m/s)

% Source positions (the targets the drones need to find)
p_sources = [20, 50; ...
             -50, 80];  

% PSO Parameters
inertia = 0.9;           % Inertia weight
cognitive = 0.8;         % Cognitive (particle's own best) factor
social = 1;            % Social (swarm best) factor
velocity_scale = 0.01;   % Scaling factor for velocity

% Initialize drone positions at the bottom-left corner of the square
positions = bounds(1) * ones(N_drones, 2);  % Initialize all drones at [-100, -100]
velocities = velocity_scale * randn(N_drones, 2);  % Small random velocities

p_best = positions;       % Each drone's best known position
p_best_values = arrayfun(@(i) combinedNSS(positions(i, :), p_sources), 1:N_drones);

% Find the global best position
[~, g_best_idx] = max(p_best_values);
g_best = p_best(g_best_idx, :);

% Visualization setup
figure;
scatter(p_sources(:,1), p_sources(:,2), 100, 'r', 'filled'); hold on;
drone_plot = scatter(positions(:,1), positions(:,2), 70, 'b', 'filled');
axis([bounds(1) bounds(2) bounds(1) bounds(2)]);
xlabel('x'); ylabel('y');
title('Drone Positions and Movement');
drawnow;

% PSO Loop with exploration-exploitation adaptation
for iter = 1:N_iterations
    % Adapt the inertia and random scaling factor
    inertia = 0.9 - (0.8 * (iter / N_iterations));  % Linearly decrease inertia from 0.9 to 0.1
    random_scale = 0.1 + (0.9 * (1 - (iter / N_iterations)));  % Reduce randomness as iteration progresses
    
    for i = 1:N_drones
        % Update velocity
        velocities(i, :) = inertia * velocities(i, :) + ...
                           cognitive * rand() * (p_best(i, :) - positions(i, :)) + ...
                           social * rand() * (g_best - positions(i, :));
                       
        % Add noise with decreasing randomness
        velocities(i, :) = velocities(i, :) + random_scale * randn(1, 2);  % Random noise

        % Cap velocity to the maximum allowable speed
        speed = norm(velocities(i, :));
        if speed > max_velocity
            velocities(i, :) = (velocities(i, :) / speed) * max_velocity;
        end
        
        % Update position
        positions(i, :) = positions(i, :) + velocities(i, :);
        
        % Keep positions within bounds
        positions(i, :) = max(min(positions(i, :), bounds(2)), bounds(1));
        
        % Evaluate the new combined NSS signal at the new position
        value = combinedNSS(positions(i, :), p_sources);
        
        % Update personal best if the new NSS value is higher
        if value > p_best_values(i)
            p_best(i, :) = positions(i, :);
            p_best_values(i) = value;
        end
    end
    
    % Update global best if any drone's personal best is better
    [~, g_best_idx] = max(p_best_values);
    g_best = p_best(g_best_idx, :);
    
    % Update the plot
    set(drone_plot, 'XData', positions(:,1), 'YData', positions(:,2));
    title(sprintf('Iteration %d', iter));
    pause(0.01)
    drawnow;
end


% Final best positions
scatter(g_best(1), g_best(2), 100, 'g', 'filled');
legend('Sources', 'Drone Positions', 'Best Estimate');
title('Final Drone Position and Global Best');
hold off;

% Combined NSS Function: Sum of the NSS from all sources
function value = combinedNSS(position, p_sources)
    value = 0;
    for i = 1:size(p_sources, 1)
        value = value + NSS(position, p_sources(i, :));
    end
end

% NSS Function: Normalized Source Strength (Signal Function)
function NSS_value = NSS(position, p_source)
    % Calculate the distance between the drone and the source
    r = sqrt(sum((position - p_source).^2));
    
    % Prevent division by zero by setting a minimum distance
    r = max(r, 1e-11);
    
    % Calculate normalized source strength (NSS) based on distance
    C = 1;  % A constant scaling factor
    NSS_value = C / r^4;  % Signal strength diminishes rapidly with distance
end

