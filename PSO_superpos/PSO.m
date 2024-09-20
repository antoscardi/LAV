clear; close all; clc;

%% Set seed for reproducibility
seed = 42;
rng(seed)

% Parameters
N_drones = 14;           % Increased number of drones for better exploration
N_iterations = 1000;      % Number of iterations
bounds = [-100, 100];    % Search space boundaries
max_velocity = 5;        % Maximum allowable velocity (m/s)

% Source positions (the targets the drones need to find)
% Source positions (the targets the drones need to find)
p_sources = [20, 50;     % Source 1
             -50, 80;    % Source 2
             60, -30;    % Source 3
             10, 90];    % Source 4
  

% PSO Parameters
inertia = 1.2;           % Inertia weight
cognitive = 2;           % Cognitive (particle's own best) factor
social = 2;              % Social (swarm best) factor
velocity_scale = 0.01;   % Scaling factor for velocity
repulsion_factor = 500; % Repulsion factor between drones

% Number of groups (e.g., 2 groups for 2 sources)
N_groups = 4;

% Initialize drone positions at random locations within bounds
positions = bounds(1) * ones(N_drones, 2);  % Initialize all drones at [-100, -100]
velocities = velocity_scale * randn(N_drones, 2);  % Small random velocities

p_best = positions;       % Each drone's best known position
p_best_values = arrayfun(@(i) combinedNSS(positions(i, :), p_sources), 1:N_drones);

% Initialize group bests (one for each group)
g_best_local = zeros(N_groups, 2); 
g_best_local_values = -Inf * ones(N_groups, 1);  % Initialize to a very low value

% Assign drones to groups based on initial random positions
group_indices = kmeans(positions, N_groups);  % Cluster drones into N_groups

% Visualization setup
figure;
scatter(p_sources(:,1), p_sources(:,2), 100, 'r', 'filled'); hold on;
drone_plot = scatter(positions(:,1), positions(:,2), 70, 'b', 'filled');
axis([bounds(1) bounds(2) bounds(1) bounds(2)]);
xlabel('x'); ylabel('y');
title('Drone Positions and Movement');
drawnow;

% PSO Loop with group-based exploration
for iter = 1:N_iterations
    % Adapt the inertia and random scaling factor
    inertia = 0.9 - (0.8 * (iter / N_iterations));  % Linearly decrease inertia from 0.9 to 0.1
    random_scale = 300 * (1 - (iter / N_iterations));  % Starts at 1000% and reduces to 0
    %repulsion_factor = 500 * (1 - (iter / N_iterations));
    
    for i = 1:N_drones
        % Update velocity with personal best and group best
        group_idx = group_indices(i);  % Get the group index for the current drone
        velocities(i, :) = inertia * velocities(i, :) + ...
                           cognitive * rand() * (p_best(i, :) - positions(i, :)) + ...
                           social * rand() * (g_best_local(group_idx, :) - positions(i, :));

        % Add repulsion from other drones
        for j = 1:N_drones
            if i ~= j
                distance = norm(positions(i, :) - positions(j, :));
                if distance > 0
                    % Repulsion force decreases with distance
                    repulsion = repulsion_factor / distance^2;
                    velocities(i, :) = velocities(i, :) + ...
                                    repulsion * (positions(i, :) - positions(j, :));
                end
            end
        end
                       
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
        disp(value)
        
        % Update personal best if the new NSS value is higher
        if value > p_best_values(i)
            p_best(i, :) = positions(i, :);
            p_best_values(i) = value;
        end
        
        % Update the group best for each group
        if value > g_best_local_values(group_idx)
            g_best_local(group_idx, :) = positions(i, :);
            g_best_local_values(group_idx) = value;
        end
    end
    
    % Reassign drones to groups based on proximity to group bests
    group_indices = kmeans(positions, N_groups);
    
    % Update the plot
    set(drone_plot, 'XData', positions(:,1), 'YData', positions(:,2));
    title(sprintf('Iteration %d', iter));
    pause(0.01)
    drawnow;
end

% Final best positions
scatter(g_best_local(:,1), g_best_local(:,2), 100, 'g', 'filled');
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
    r = max(r, 1e-5);
    
    % Calculate normalized source strength (NSS) based on distance
    C = 1;  % A constant scaling factor
    NSS_value = C / r^4;  % Signal strength diminishes rapidly with distance
end