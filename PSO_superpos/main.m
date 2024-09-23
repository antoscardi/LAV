clear; close all; clc;
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
n_drones = 4;           % Number of drones in the swarm. Each drone acts as a particle in the PSO algorithm. The 
                         % drones will search the space to find the sources.

n_iterations = 500;      % Total number of iterations for the PSO algorithm. This controls how long it will run.

bounds = [-100, 100];    % Search space boundaries for drone positions. This defines the limits for the x and y 
                         % coordinates within which the drones can move. Example: drones can move in a square area 
                         % from (-100, -100) to (100, 100).

max_velocity = 1.5;      % Maximum allowable velocity for each drone (m/s). Limits how fast a drone can move within 
                         % the search space, preventing overshooting the target.

% Source positions (the targets the drones need to find)
p_sources = [20, 50;     % Coordinates of Source 1 (x, y).
             -50, 80;    % Coordinates of Source 2 (x, y).
             60, -30;    % Coordinates of Source 3 (x, y).
             10, 90];    % Coordinates of Source 4 (x, y). These are the fixed positions the drones are searching 
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                 INITIALIZATION                                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize group bests (one for each group)
group_best_positions = zeros(n_groups, 2);     % Initialize group best positions as zero.
group_best_values = -Inf * ones(n_groups, 1);  % Initialize group best values to a very low value (-Inf).
initial_positions = zeros(n_drones, 2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The following lines ensures that each source (n_groups) gets at least one drone assigned from a pool of n_drones. 
% The assignment is random but guarantees that if n_drones is equal to n_groups, each source gets one drone. 
% If n_drones > n_groups, additional drones are randomly assigned to the existing sources. 
% The resulting group indices are shuffled to randomize the order.
if n_drones >= n_groups
    % If there are enough or more drones than sources
    group_indices = (1:n_groups)';  % Assign each source one drone
    if n_drones > n_groups
        % Randomly assign remaining drones if there are more drones than sources
        additional_assignments = randi(n_groups, n_drones - n_groups, 1);  
        group_indices = [group_indices; additional_assignments];  % Concatenate as column vectors
    end
else
    % If there are fewer drones than sources, create indices that go from 1 to n_drones 
    group_indices = (1:n_drones)'; 
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize ARTVA
artva = ARTVAs();

% Initialize the particles array using a cell array
particles = cell(n_drones, 1); 

% Initialize Drones and their initial goal
angles = zeros(1, n_drones);    % Array to store the angles of each drone
omega = 360 / n_drones;         % Divide 360 degrees by the number of drones
max_distance = 50;

for i = 1:n_drones
    % Calculate the angle for each drone in degrees
    omega_i = omega * (i - 1);
    angles(i) = omega_i;
    % Calculate the slope for the drone's path based on the angle
    m = tan(deg2rad(omega_i));
    % Assign a goal for each drone based on the angle
    if (omega_i > 315 && omega_i <= 360) || omega_i <= 45
        % First and fourth quadrant (x positive, y positive/negative)
        goal = [max_distance, m * max_distance];
    elseif omega_i > 45 && omega_i <= 135
        % Second quadrant (x positive, y positive)
        goal = [max_distance / m, max_distance];
    elseif omega_i > 135 && omega_i <= 225
        % Third quadrant (x negative, y negative)
        goal = [-max_distance, m * -max_distance];
    elseif omega_i > 225 && omega_i <= 315
        % Fourth quadrant (x negative, y negative)
        goal = [-max_distance / m, -max_distance];
    end
    particles{i} = Particle(goal, velocity_randomness, max_velocity, bounds, group_indices(i), i);  % Initialize particle
    initial_positions(i, :) = particles{i}.position; 
end

% Initialize the Plotter
plotter = Plotter(p_sources, bounds, n_drones, group_indices, n_groups, initial_positions);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                        EXPLORATION PHASE                                                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% During this phase the drones move in straight lines, it can be implemented in the circle case or in the radial.
for iter = 1:ceil(max_distance / max_velocity)
    for i = 1:n_drones
        particle = particles{i};

        % Calculate direction to move in a straight line towards the goal
        direction = (particle.goal - particle.position) / norm(particle.goal - particle.position);
        
        % Set velocity to move towards the goal
        particle.velocity = direction * max_velocity;
        
        % Update position (no randomness, no inertia, just move straight)
        particle = particle.update_position();  % Update based on velocity
        particles{i} = particle;
        
        % Debug: Display current position and goal
        fprintf('Iteration %d, Drone %d: Position = [%.2f, %.2f], Goal = [%.2f, %.2f]\n', ...
                iter, i, particle.position(1), particle.position(2), particle.goal(1), particle.goal(2));
    end
    % Update plot with current drone positions
    positions = zeros(n_drones, 2);  % Initialize a matrix to store positions
    for i = 1:n_drones
        positions(i, :) = particles{i}.position;  % Extract position of each particle
    end
    plotter.draw(positions, iter, group_indices);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                        MAIN PARTICLE SWARM OPTIMIZATION LOOP                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize exclusion zones as an empty matrix to store the positions of discovered sources
exclusion_zones = [];  % Will store positions of found sources

% Set parameters for exclusion zone behavior
exclusion_radius = 5;       % Distance around the source where the exclusion zone starts
repulsion_strength = 100;     % Strength of repulsive force
for iter = 1:n_iterations
    %inertia = inertia_initial - (iter / n_iterations)^4;
    inertia = inertia_initial;
    %repulsion_factor = repulsiion_initial * (1 - (iter / n_iterations)^2);  % Decrease repulsion over time
    
    for i = 1:n_drones
        particle = particles{i};
        group_idx = particle.group_idx;
        %fprintf('Iteration: %d, Particle: %d, Group: %d, Position: [%.1f, %.1f]\n', ...
        %    iter, particle.identifier, group_idx, particle.position(1), particle.position(2));  % Debug print       
        
        % Evaluate NSS value at the new position
        particle = particle.evaluate_nss(@artva.superpositionNSS, p_sources);

        % Update group best if necessary
        if particle.nss_value > group_best_values(group_idx)
            group_best_positions(group_idx, :) = particle.position;
            group_best_values(group_idx) = particle.nss_value;
        end

        % Check if NSS value is greater than 500 (i.e., the drone has found a source)
        if particle.nss_value > 500
            % Add this source position to the exclusion zone
            exclusion_zones = [exclusion_zones; group_best_positions(group_idx, :)];
        end

        % Update velocity with personal and group best
        particle = particle.update_velocity(group_best_positions(group_idx, :), inertia, ...
            cognitive_factor, social_factor);

        % Update particle position and apply boundary constraints
        particle = particle.update_position();
        % Reassign the particles after the update
        particles{i} = particle;
    end

    % Update plot with current drone positions
    positions = zeros(n_drones, 2);  % Initialize a matrix to store positions
    for i = 1:n_drones
        positions(i, :) = particles{i}.position;  % Extract position of each particle
    end
    plotter.draw(positions, iter, group_indices);
end

% Final plot with group best positions

plotter.plot_best(group_best_positions, group_indices);


function intra_repulsion_force = calculate_intra_cluster_repulsion(positions, group_indices, repulsion_factor)
    n_drones = size(positions, 1);  % Number of drones
    intra_repulsion_force = zeros(n_drones, 2);  % Initialize the intra-cluster repulsion force for each drone
    
    % Loop over each drone to calculate intra-cluster repulsion forces
    for i = 1:n_drones
        for j = 1:n_drones
            if i ~= j && group_indices(i) == group_indices(j)  % Same group
                distance = norm(positions(i, :) - positions(j, :));
                if distance > 0
                    % Repulsion force decreases with the square of the distance
                    repulsion = repulsion_factor / distance^2;
                    intra_repulsion_force(i, :) = intra_repulsion_force(i, :) + ...
                                                  repulsion * (positions(i, :) - positions(j, :)) / distance;
                end
            end
        end
    end
end
