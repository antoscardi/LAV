clear; close all; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                         PARTICLE SWARM OPTIMIZATION                                            %
% This algorithm needs to ensure finding the number of sources if we have enough drones for each source          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set seed for reproducibility
global seed;
seed = 42;
rng(seed)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                              HYPERPARAMETERS                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n_drones = 4;           % Number of drones in the swarm. Each drone acts as a particle in the PSO algorithm. The 
                         % drones will search the space to find the sources.

n_iterations = 200;     % Total number of iterations for the PSO algorithm. This controls how long it will run.

bounds = [-100, 100];    % Search space boundaries for drone positions. This defines the limits for the x and y 
                         % coordinates within which the drones can move. Example: drones can move in a square area 
                         % from (-100, -100) to (100, 100).

max_velocity = 1.5;        % Maximum allowable velocity for each drone (m/s). Limits how fast a drone can move within 
                         % the search space, preventing overshooting the target.

% Source positions (the targets the drones need to find)
p_sources = [20, 50;     % Coordinates of Source 1 (x, y).
             -50, 80;    % Coordinates of Source 2 (x, y).
             60, -30;    % Coordinates of Source 3 (x, y).
             10, 90];    % Coordinates of Source 4 (x, y). These are the fixed positions the drones are searching 
                         % for in the search space.
n_sources = size(p_sources, 1);

% PSO Parameters
inertia_weight = 1.2;    % Inertia weight, controls how much of the drone's previous velocity is retained. Higher 
                         % inertia promotes exploration, while lower values promote faster convergence.

cognitive_factor = 2;    % Cognitive factor (personal learning coefficient), governs how much a drone is attracted 
                         % to its own best-known position. Higher values make drones focus on their personal best.

social_factor = 2;       % Social factor (global learning coefficient), controls how much a drone is influenced by 
                         % the swarm's global best-known position. Higher values increase the influence of the swarm.

velocity_scale = 0.1;   % Scaling factor for velocity to control the effect of inertia, cognitive, and social 
                         % factors on the drone's velocity. Keeps velocity updates reasonable in magnitude.

repulsion_factor = 500;  % Repulsion factor between drones to avoid clustering. Helps drones spread out to explore 
                         % different regions by adding a repulsion force between them.

% Grouping parameters
n_groups = n_sources;    % Number of groups (or clusters) for the drones. Each group is associated with one of the 
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

% Step 1: Start by assigning drones to sources
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

% Step 2: Shuffle the assignments to make the order random
group_indices = group_indices(randperm(n_drones));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize ARTVA
artva = ARTVAs();

% Initialize the particles array using a cell array
particles = cell(n_drones, 1); 

% Initialize Drones
for i = 1:n_drones
    particles{i} = Particle(velocity_scale, bounds, group_indices(i), i);  % Initialize particle
    initial_positions(i, :) = particles{i}.position;  % Store the 2D position (x, y) of each particle
end

% Initialize the Plotter
plotter = Plotter(p_sources, bounds, n_drones, group_indices, n_groups, initial_positions);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                        MAIN PARTICLE SWARM OPTIMIZATION LOOP                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for iter = 1:n_iterations
    inertia = inertia_weight - (iter / n_iterations)^2;  % Much slower decrease over time
    repulsion_factor = repulsion_factor * (1 - (iter / n_iterations)^2);  % Decrease repulsion over time
    
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

        % Update velocity with personal and group best
        particle = particle.update_velocity(group_best_positions(group_idx, :), inertia, ...
            cognitive_factor, social_factor, max_velocity);       

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
