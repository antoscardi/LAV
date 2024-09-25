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
n_drones = 8;            % Number of drones in the swarm. Each drone acts as a particle in the PSO algorithm. The 
                         % drones will search the space to find the sources.

n_iterations = 500;      % Total number of iterations for the PSO algorithm. This controls how long it will run.

bounds = [-100, 100];      % Search space boundaries for drone positions. This defines the limits for the x and y 
                         % coordinates within which the drones can move. Example: drones can move in a square area 
                         % from (-100, -100) to (100, 100).

max_velocity = 1.5;      % Maximum allowable velocity for each drone (m/s). Limits how fast a drone can move within 
                         % the search space, preventing overshooting the target.

% Source positions (the targets the drones need to find)
p_sources = [20, 50;     % Coordinates of Source 1 (x, y).
             -50, 70;    % Coordinates of Source 2 (x, y).
             60, -30;    % Coordinates of Source 3 (x, y).
             -10, 30];   % Coordinates of Source 4 (x, y). These are the fixed positions the drones are searching 
                         % for in the search space.
n_sources = size(p_sources, 1);

% PSO Parameters
inertia_initial = 1.01;     % Inertia weight, controls how much of the drone's previous velocity is retained. Higher 
                             % inertia promotes exploration, while lower values promote faster convergence.
                             % Typical range: [1 - 1.4]

cognitive_factor = 2.5;      % Cognitive factor (personal learning coefficient), governs how much a drone is attracted 
                             % to its own best-known position. Higher values make drones focus on their personal best.
                             % Typical range: [1.5 - 2.5]

social_factor = 1.5;         % Social factor (global learning coefficient), controls how much a drone is influenced by 
                             % the swarm's global best-known position. Higher values increase the influence of the swarm.
                             % Typical range: [1.5 - 2.5], in our case the swarm is relative to the group.

repulsiion_initial = 1;      % Repulsion factor between drones to avoid clustering. Helps drones spread out to explore 
                             % different regions by adding a repulsion force between them.

velocity_randomness = 0.2;   % Factor between 0 and 1 that controls the amount of randomness added to particle velocity.
                             % A higher value increases exploration by adding more variation to the drone's movement,while
                             % a lower value reduces randomness, promoting more predictable movement towards the target.

n_groups = n_sources;        % Number of groups (or clusters) for the drones. Each group is associated with one of the 
                             % sources. For example, if there are 4 sources, drones can be divided into 4 groups to 
                             % focus on different sources.

communication_radius = 5;    % Distance between two drones that enables communication with each other.

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
particles = init_drones(n_drones, bounds, group_indices, max_velocity, velocity_randomness); 

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
        
        % Set velocity to move towards the goal
        particle.velocity = direction * max_velocity;
        
        % Update position (no randomness, no inertia, just move straight)
        particle = particle.update_position();  

        % Extract position of each particle
        positions(i, :) = particles{i}.position;  
    end
    % Update plot with current drone positions
    plotter.draw(positions, iter, group_indices);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                        MAIN PARTICLE SWARM OPTIMIZATION LOOP                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize exclusion zones as an empty matrix to store the positions of discovered sources
for iter = iter:n_iterations
    %inertia = inertia_initial - (iter / n_iterations)^4;
    inertia = inertia_initial;
    
    for i = 1:n_drones
        particle = particles{i};
        group_idx = particle.group_idx;

        % Only update group best if the particle is not in an exclusion zone
        if isempty(particle.shared_exclusion_zones) || ~particle.is_in_exclusion_zone
            % Evaluate NSS value at the position
            particle = particle.evaluate_nss(@artva.superpositionNSS, p_sources);

            % Update group best if necessary
            if particle.nss_value > group_best_values(group_idx)
                group_best_positions(group_idx, :) = particle.position;
                group_best_values(group_idx) = particle.nss_value;
            end
        else
            % If a particle moves out of an exclusion zone, reset the group best
            fprintf('Drone %d moved out of an exclusion zone. Resetting group best for group %d.\n', particle.identifier, group_idx);
            group_best_values(group_idx) = -Inf;  % Reset the group best value
        end

        % Update velocity with personal and group best
        particle = particle.update_velocity(group_best_positions(group_idx, :), inertia, ...
            cognitive_factor, social_factor);

        % Update particle position and apply boundary constraints
        particle = particle.update_position();

        % Loop through other drones
        for j = i+1:n_drones  % Start at i+1 to avoid double-checking
            other_particle = particles{j};
            % The exclusion zones only apply to particles of different groups
            if particle.group_idx ~= other_particle.group_idx
                % Check if particle and other_particle are within communication radius
                if particle.can_I_communicate_with(other_particle, communication_radius)
                    %fprintf('Iteration %d: Drone %d and Drone %d are within the communication radius of %d units\n', ...
                        %iter, particle.identifier, other_particle.identifier, communication_radius);
                    % Check if sharing has already happened
                    if ~particle.has_shared_matrix(j) && (particle.victim_found_flag || other_particle.victim_found_flag)
                        % First case: particle has found a victim and shares its exclusion zone
                        if particle.victim_found_flag
                            plotter.plot_exclusion_zone(2*particle.exclusion_zone_radius, particle.my_exclusion_zone);
                            particle = particle.share_exclusion_zones(other_particle);  % Share exclusion zones from particle to other_particle
                        end
                        % Second case: other_particle has found a victim and shares its exclusion zone
                        if other_particle.victim_found_flag
                            plotter.plot_exclusion_zone(2*other_particle.exclusion_zone_radius, other_particle.my_exclusion_zone)
                            other_particle = other_particle.share_exclusion_zones(particle);  % Share exclusion zones from other_particle to particle
                        end
                        % Mark that sharing has been done for both drones
                        particle.has_shared_matrix(j) = true;
                        other_particle.has_shared_matrix(i) = true;
                    end
                end
            end
        end

        % After sharing, check if the drone is in an exclusion zone and move away if necessary
        particle = particle.check_if_in_exclusion_zone();

        % Extract position of each particle
        positions(i, :) = particles{i}.position;  
    end
    % Update plot with current drone positions
    plotter.draw(positions, iter, group_indices);
end

% Final plot with group best positions
plotter.plot_best(group_best_positions, group_indices);

