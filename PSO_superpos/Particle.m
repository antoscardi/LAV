%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                               PARTICLE CLASS                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The Particle class represents each individual drone in the Particle Swarm Optimization (PSO) algorithm.
% Each particle stores its current position and velocity, as well as its personal best position and the associated
% best NSS value (signal strength) found so far.
%
% Properties:
% - position: The current position of the particle (x, y).
% - velocity: The current velocity of the particle (x, y).
% - p_best: The best position found by the particle so far.
% - nss_best_value: The highest NSS value achieved at the personal best position.
% - nss_value: The current NSS value at the particle's current position.
% - bounds: The boundaries within which the particle must stay.
%
% Methods:
% - update_velocity: Updates the velocity based on cognitive, social factors, and repulsion from other particles.
% - update_position: Updates the particle’s position based on the current velocity and applies boundary constraints.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef Particle < handle
    properties
        position              % Current position of the particle (x, y)
        velocity              % Current velocity of the particle (x, y)
        p_best                % Best known position of the particle
        nss_best_value        % Best known NSS value at the personal best position
        nss_value             % Current NSS value at the particle's position
        bounds                % Boundaries of the search space
        group_idx             % Group index of the particle, indicating which group (source) the particle is assigned to
        identifier            % Unique identifier for the particle, used to distinguish it from other particles
        velocity_randomness   % Factor controlling the amount of randomness added to the particle's velocity updates
        max_velocity          % Maximum allowed speed for the particle to prevent overly large velocity updates
        goal                  % First goal they need to reach
        victim_found_flag     % When the drone has found a source sets this flag to true
        my_exclusion_zone     % Exclusion zones created by this drone
        shared_exclusion_zones % Exclusion zones shared by other drones 
        has_shared_matrix     % A matrix to track if exclusion zones have been shared with other drones
        exclusion_zone_radius % How big is the exclusion zone
        is_in_exclusion_zone

    end
    
    methods
        % Constructor to initialize particle with position, velocity, and bounds
        function obj = Particle(goal, velocity_randomness, max_velocity, bounds, group_idx, identifier, n_drones)
            obj.max_velocity = max_velocity;
            %obj.position =  [bounds(1), bounds(1)];   % Initialize all drones at the lower bound [-100, -100].;
            obj.position = [0, 0];                     % Initialize all drones at the center
            obj.velocity = velocity_randomness * rand(1,2);
            if norm(obj.velocity) > obj.max_velocity
                obj.velocity = (obj.velocity / norm(obj.velocity)) * obj.max_velocity;
            end  
            obj.p_best = obj.position;  % Each drone's best-known position starts at its initial position.
            obj.nss_best_value = -Inf;  % Initial best value set to a very low number
            obj.bounds = bounds;
            obj.group_idx = group_idx;
            obj.identifier = identifier;
            obj.velocity_randomness = velocity_randomness;
            obj.goal = goal;
            obj.victim_found_flag = false;
            obj.has_shared_matrix = false(n_drones, 1);
            obj.exclusion_zone_radius = 2; % The NSS decreases as r^4 so after 2 meters is 0.125
            fprintf('Drone initialized at pos: [%1d, %1d] and vel: [%.1f, %.1f]  with group index: %d and Goal :[%.1f, %.1f]\n', ...
                obj.position(1), obj.position(2), obj.velocity(1), obj.velocity(2),obj.group_idx, obj.goal(1), obj.goal(2));
        end

        % Update the particle's velocity based on personal and global best
        function obj = update_velocity(obj, g_best_local, inertia, cognitive, social)
            % Personal and social components
            obj.velocity = inertia * obj.velocity + ...
                           cognitive * rand() * (obj.p_best - obj.position) + ...
                           social * rand() * (g_best_local - obj.position);

            % Add random noise to velocity between [-1, 1]
            obj.velocity = obj.velocity + obj.velocity_randomness * (2 * rand(1, 2) - 1);

            % Cap velocity to max allowable speed
            if norm(obj.velocity) > obj.max_velocity
                obj.velocity = (obj.velocity / norm(obj.velocity)) * obj.max_velocity;
            end     
            %fprintf('Particle %d velocity updated to: [%.1f, %.1f]\n',obj.identifier, obj.velocity(1), obj.velocity(2));  % Debug print
        end
        
        % Update particle's position based on velocity and apply boundary conditions
        function obj = update_position(obj)
            obj.position = obj.position + obj.velocity;
            % Enforce boundaries
            obj.position(1) = max(min(obj.position(1), obj.bounds(2)), obj.bounds(1));
            obj.position(2) = max(min(obj.position(2), obj.bounds(2)), obj.bounds(1));
            %fprintf('Particle %d position updated to: [%.1d, %.1d]\n',obj.identifier,  obj.position(1), obj.position(2));  % Debug print
        end
        
        % Evaluate NSS value at the current position and update personal best if needed
        function obj = evaluate_nss(obj, superpositionNSS, p_sources)
            obj.nss_value = superpositionNSS(obj.position, p_sources);
            if obj.nss_value > obj.nss_best_value
                obj.p_best = obj.position;
                obj.nss_best_value = obj.nss_value;
            end
            if obj.nss_value > 500 && ~obj.victim_found_flag && isempty(obj.my_exclusion_zone)
                obj.victim_found_flag = true;
                obj.my_exclusion_zone = obj.position;  % Add current position as my exclusion zone
                fprintf('Drone %d has found a source at position [%.1f, %.1f]\n', ...
                    obj.identifier, obj.position(1), obj.position(2));
            end
            %fprintf('Particle %d NSS evaluated at: %.1d\n', obj.identifier, obj.nss_value);  % Debug print
        end

        % Check if two particles are within the communication radius
        function is_within_radius = can_I_communicate_with(obj, other_particle, communication_radius)
            % Calculate the distance between the two particles
            distance = norm(obj.position - other_particle.position);
            % Check if the distance is less than or equal to the communication radius
            is_within_radius = distance <= communication_radius;
        end

        % Share own exclusion zone info with another drone
        function obj = share_exclusion_zones(obj, other_particle)
            % Share this drone's own exclusion zone with the other drone
            other_particle.shared_exclusion_zones = [other_particle.shared_exclusion_zones; obj.my_exclusion_zone];
            
            % Remove duplicate exclusion zones (ensure unique rows)
            other_particle.shared_exclusion_zones = unique(other_particle.shared_exclusion_zones, 'rows');
            fprintf('Drone %d shared its exclusion zone: [%.1f, %.1f] with Drone %d\n', ...
                     obj.identifier, obj.my_exclusion_zone(1), obj.my_exclusion_zone(2), other_particle.identifier);
            fprintf('Drone %d now has the following shared exclusion zones: [%.1f, %.1f] \n', ...
                     other_particle.identifier, other_particle.shared_exclusion_zones(1), other_particle.shared_exclusion_zones(2));
            disp(other_particle.shared_exclusion_zones);
        end

        % Function to check if the drone is in any shared exclusion zone (shared by others)
        function obj = check_if_in_exclusion_zone(obj)
            obj.is_in_exclusion_zone = false;
            % Iterate through all shared exclusion zones
            if obj.shared_exclusion_zones
                for k = 1:size(obj.shared_exclusion_zones, 1)
                    exclusion_zone = obj.shared_exclusion_zones(k, :);  % Exclusion zone position
                    distance_to_zone = norm(obj.position - exclusion_zone);
                    
                    % Check if the drone is within the exclusion zone radius (e.g., 5 units)
                    if distance_to_zone < obj.exclusion_zone_radius
                        % Set the flag to indicate the drone is in an exclusion zone
                        obj.is_in_exclusion_zone = true;
                        % Move away from the exclusion zone
                        obj = obj.move_away_from_exclusion(exclusion_zone);
                        fprintf('Drone %d is in a shared exclusion zone and moving away.\n', obj.identifier);
                        % Introduce randomness to encourage exploration
                        obj.velocity_randomness = 1;
                        randomness_factor = 1 * (rand(1, 2) - 0.5);  % Strong random boost to force exploration
                        obj.position = obj.position + randomness_factor;  % Apply random perturbation
                        
                        % Reset personal best (P-best) to a far random position to avoid the exclusion zone
                        random_position_far = obj.position + 3 * (rand(1, 2) - 0.5);  % Set a far random position as new P-best
                        obj.p_best = random_position_far;
                        obj.nss_best_value = -Inf;  % Reset the best NSS value to a very low number
                        
                        % Optionally, reduce inertia or reset velocity to encourage exploration
                        obj.velocity = zeros(1, 2);  % Reset velocity for a fresh search
                    end
                end
            end
        end

        % Function to move the drone away from the exclusion zone
        function obj = move_away_from_exclusion(obj, exclusion_zone)
            % Calculate the direction away from the exclusion zone
            direction_away = obj.position - exclusion_zone;
            direction_away = direction_away / norm(direction_away);  % Normalize the direction vector
            
            % Update the position of the drone by moving it a small step away
            step_size = 100;  % Set how far the drone should move per iteration
            obj.position = obj.position + step_size * direction_away;
        end
    end
end

