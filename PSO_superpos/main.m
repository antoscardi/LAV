clear; close all; clc;
addpath("functions");
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                         PARTICLE SWARM OPTIMIZATION                                            %
% This algorithm needs to ensure finding the number of sources if we have enough drones for each source          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set seed for reproducibility
seed = 42;
rng(seed)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                              HYPERPARAMETERS                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n_drones = 3;            % Number of drones in the swarm. Each drone acts as a particle in the PSO algorithm. The 
                         % drones will search the space to find the sources.

n_iterations = 500;      % Total number of iterations for the PSO algorithm. This controls how long it will run.

bounds = [-100, 100];      % Search space boundaries for drone positions. This defines the limits for the x and y 
                         % coordinates within which the drones can move. Example: drones can move in a square area 
                         % from (-100, -100) to (100, 100).

max_velocity = 1.5;      % Maximum allowable velocity for each drone (m/s). Limits how fast a drone can move within 
                         % the search space, preventing overshooting the target.

% Source fixed positions (the targets the drones need to find)
p_sources = [ 20, 50;     % Coordinates of Source 1 (x, y).
              5, 70;
              20, 52];     % Coordinates of Source 2 (x, y).   
n_sources = size(p_sources, 1);

% PSO Parameters
inertia = 1.02;              % Inertia weight, controls how much of the drone's previous velocity is retained. Higher 
                             % inertia promotes exploration, while lower values promote faster convergence.
                             % Typical range: [1 - 1.4]

cognitive_factor = 1;        % Cognitive factor (personal learning coefficient), governs how much a drone is attracted 
                             % to its own best-known position. Higher values make drones focus on their personal best.
                             % Typical range: [1.5 - 2.5]

social_factor = 1;           % Social factor (global learning coefficient), controls how much a drone is influenced by 
                             % the swarm's global best-known position. Higher values increase the influence of the swarm.
                             % Typical range: [1.5 - 2.5], in our case the swarm is relative to the group.

velocity_randomness = 0.2;   % Factor between 0 and 1 that controls the amount of randomness added to particle velocity.
                             % A higher value increases exploration by adding more variation to the drone's movement,while
                             % a lower value reduces randomness, promoting more predictable movement towards the target.

n_groups = n_sources;        % Number of groups (or clusters) for the drones. Each group is associated with one of the 
                             % sources. For example, if there are 4 sources, drones can be divided into 4 groups to 
                             % focus on different sources.

communication_radius = 5;    % Distance between two drones that enables communication with each other.

step_size = 40;              % Set how far the drone should move away in ONLY ONE ITERATION

exclusion_zone_radius = 1; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                 INITIALIZATION                                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize group bests (one for each group)
group_best_positions = zeros(n_groups, 2);     % Initialize group best positions as zero.
group_best_values = -Inf * ones(n_groups, 1);  % Initialize group best values to a very low value (-Inf).
positions = zeros(n_drones, 2);                % Initialize a matrix to store changing positions at each iteration.

% Initialize drones in different groups.
group_indices = sort_drones_in_groups(n_drones, n_groups);

% Initialize ARTVA
artva = ARTVAs();

% Initialize the particles array using a cell array
particles = init_drones(exclusion_zone_radius, n_drones, bounds, group_indices, max_velocity, velocity_randomness); 

% Initialize the Plotter
plotter = Plotter(p_sources, bounds, n_drones, group_indices);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                        EXPLORATION PHASE                                                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% During this phase the drones move in straight lines, it can be implemented in the circle case or in the radial.
% We choose the circle because it has been shown to be the best way to equally partition the space.
for iter = 1:ceil(bounds(2)/2 / max_velocity)
    for i = 1:n_drones
        particle = particles{i};
        % Calculate direction to move in a straight line towards the goal
        direction = (particle.goal - particle.position) / norm(particle.goal - particle.position);
        particle.velocity = direction * max_velocity; % Move at maximum velocity
        particle.update_position();  
        positions(i, :) = particles{i}.position;  
    end
    % Update plot with current drone positions
    plotter.draw(positions, iter);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                        MAIN PARTICLE SWARM OPTIMIZATION LOOP                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize exclusion zones as an empty matrix to store the positions of discovered sources
for iter = iter:n_iterations
    %inertia = inertia_initial - (iter / n_iterations)^4;
    
    for i = 1:n_drones
        particle = particles{i};
        group_idx = particle.group_idx;

        % Update velocity with personal and group best
        particle.update_velocity(group_best_positions(group_idx, :), inertia, ...
            cognitive_factor, social_factor);

        % Update particle position and apply boundary constraints
        particle.update_position();

        % Loop through other drones
        for j = i+1:n_drones  % Start at i+1 to avoid double-checking
            other_particle = particles{j};
            % Check if particle and other_particle are within communication radius
            if particle.can_I_communicate_with(other_particle, communication_radius)
                % Check if sharing has already happened
                if ~particle.has_shared_matrix(j) && (particle.victim_found_flag || other_particle.victim_found_flag)
                    % First case: particle has found a victim and shares its exclusion zone
                    if particle.victim_found_flag
                        plotter.plot_exclusion_zone(2*particle.exclusion_zone_radius, particle.my_exclusion_zone);
                        % Share exclusion zones from particle to other_particle
                        particle = particle.share_exclusion_zones(other_particle);
                        % Remove randomness and reduce max velocity
                        particle.velocity_randomness = 0;
                        particle.max_velocity = 0.5;
                    end
                    % Second case: other_particle has found a victim and shares its exclusion zone
                    if other_particle.victim_found_flag
                        plotter.plot_exclusion_zone(2*other_particle.exclusion_zone_radius, other_particle.my_exclusion_zone)
                        % Share exclusion zones from other_particle to particle
                        other_particle = other_particle.share_exclusion_zones(particle);
                        % Remove randomness
                        other_particle.velocity_randomness = 0;
                        other_particle.max_velocity = 0.5;
                    end
                    % Mark that sharing has been done for both drones
                    particle.has_shared_matrix(j) = true;
                    other_particle.has_shared_matrix(i) = true;
                end
            end
        end

        % After sharing, check if the drone is in an exclusion zone and move away if necessary
        [is_in_exclusion_zone, which_one] = particle.check_if_in_exclusion_zone();
        if is_in_exclusion_zone
            % Check if the drone has other drones in it's group
            drones_in_same_group = sum(cellfun(@(p) p.group_idx == group_idx, particles));
            % If the drone is not alone in its group, assign a new group
            if drones_in_same_group > 1
                new_group_idx = max(group_indices) + 1;  % Assign a new group index (next available)
                particle.group_idx = new_group_idx;
                n_groups = n_groups + 1;
                % Update group_indices to include the new group
                group_indices(i) = new_group_idx;
                % Add a new entry for the new group's best position and value
                group_best_positions(new_group_idx, :) = particle.position;   % Initialize with the current position
                group_best_values(new_group_idx) = -Inf;                      % Initialize best value to -Inf
                % Update the plot
                plotter.update_drone_color(particle, group_indices);
                fprintf('Drone %d has left Group %d, and is now in new Group %d, moving away the exclusion zone.\n', ...
                particle.identifier, group_idx, new_group_idx);
            else
                % If the drone is alone, it remains in the same group
                fprintf('Drone %d is alone in Group %d, no group change needed.\n', particle.identifier, group_idx);
            end
            % Reset the group best
            fprintf('Drone %d moved out of an exclusion zone. Resetting group best for group %d.\n', particle.identifier, group_idx);
            % Move away from the exclusion zone
            particle.move_away_from_exclusion(particle.shared_exclusion_zones(which_one, :), step_size);
            % Increase the iteration number according to the step_size
            fprintf('Increasing iteration from %d to %d simulate freezing other drones.\n', iter, iter + step_size);
            iter = iter + step_size; %#ok<FXSET>
            % Introduce randomness to the new position
            randomness_factor = 10*rand(1, 2);  % Strong random boost to force exploration
            particle.position = particle.position + randomness_factor;  % Apply random perturbation
            % Reset personal best (P-best) to a far random position to avoid the exclusion zone
            random_position_far = particle.position + 100 * (2 * rand(1, 2) - 1);  % Set a far random position as new P-best
            particle.p_best = random_position_far;
            % Reset the group best
            group_best_values(group_idx) = -Inf;
            % Reset the NSS value
            particle.nss_best_value = -Inf;   % Reset the best NSS value to a very low number
            % Reset velocity to encourage exploration
            particle.velocity = zeros(1, 2);  % Reset velocity for a fresh search
        else
            % Evaluate NSS value at the position
            particle.evaluate_nss(@artva.superpositionNSS, p_sources);
            % Update group best if necessary
            if particle.nss_value > group_best_values(group_idx)
                group_best_positions(group_idx, :) = particle.position;
                group_best_values(group_idx) = particle.nss_value;
            end
        end

        % Extract position of each particle
        positions(i, :) = particles{i}.position;  
    end
    % Update plot with current drone positions
    plotter.draw(positions, iter);
end

% Final plot with group best positions
plotter.plot_best(group_best_positions, group_indices);

% Compute and print errors
max_error = 10;
compute_errors(p_sources, group_best_positions, max_error)

