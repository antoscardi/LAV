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
        position               % Current position of the particle (x, y)
        velocity               % Current velocity of the particle (x, y)
        p_best                 % Best known position of the particle
        nss_best_value         % Best known NSS value at the personal best position
        nss_value              % Current NSS value at the particle's position
        bounds                 % Boundaries of the search space
        group_idx              % Group index of the particle, indicating which group (source) the particle is assigned to
        identifier             % Unique identifier for the particle, used to distinguish it from other particles
        velocity_randomness    % Factor controlling the amount of randomness added to the particle's velocity updates
        max_velocity           % Maximum allowed speed for the particle to prevent overly large velocity updates
        goal                   % First goal they need to reach
        victim_found_flag      % When the drone has found a source sets this flag to true
        my_exclusion_zone      % Exclusion zones created by this drone
        shared_exclusion_zones % Exclusion zones shared by other drones 
        has_shared_matrix      % A matrix to track if exclusion zones have been shared with other drones
        exclusion_zone_radius  % How big is the exclusion zone
        inertia                % each particle has it s own inertia
        controller             % each particle is controlled by a controller
        rpy
        pqr
    end

    methods
        % Constructor to initialize particle with position, velocity, and bounds
        function obj = Particle(exclusion_zone_radius, goal, velocity_randomness, max_velocity, bounds, group_idx, identifier, n_drones, inertia)
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
            obj.inertia = inertia;
            obj.exclusion_zone_radius = exclusion_zone_radius; % The NSS decreases as r^4 so after 2 meters is 0.125
            obj.controller = Controller();  % Initialize the controller
            obj.rpy = [0,0,0];
            obj.pqr = [0,0,0];
            fprintf('Drone initialized at pos: [%1d, %1d] and vel: [%.1f, %.1f]  with group index: %d and Goal :[%.1f, %.1f]\n', ...
                obj.position(1), obj.position(2), obj.velocity(1), obj.velocity(2),obj.group_idx, obj.goal(1), obj.goal(2));
        end

        % Update the particle's velocity based on personal and global best
        function pso_velocity = update_velocity(obj, g_best_local, cognitive, social)
            % Personal and social components
            pso_velocity = obj.inertia * obj.velocity + ...
                           cognitive * rand() * (obj.p_best - obj.position) + ...
                           social * rand() * (g_best_local - obj.position);
            %disp(obj.velocity)
            % Add random noise to velocity 
            pso_velocity = pso_velocity + obj.velocity_randomness * (2 * rand(1, 2) - 1) * norm(pso_velocity);

            % Cap velocity to max allowable speed
            if norm(pso_velocity) > obj.max_velocity
                pso_velocity = (pso_velocity / norm(pso_velocity)) * obj.max_velocity;
            end

            % Update PSO velocity
            if any(isnan(pso_velocity)) || any(isinf(pso_velocity))
                pso_velocity = [0, 0]; % Replace with zeros
            end

            %fprintf('Drone %d has velocity [%.1f, %.1f]\n', ...
            %        obj.identifier, obj.velocity(1), obj.velocity(2));     
        end
        
        % Update particle's position based on velocity and apply boundary conditions
        function [state_dot, rotor_velocities] = update_state(obj, dt, state, desired_position, desired_velocity, desired_acceleration)

            % Controller inputs
            [input_torques, input_force] = obj.controller.control_laws(state, desired_position, desired_velocity, ...
            desired_acceleration);

            % Compute rotor velocities
            rotor_velocities = obj.controller.compute_rotor_velocities(input_force, input_torques);

            % Use the controller to compute the state derivative (new state)
            state_dot = obj.controller.quadrotor_full_dynamics(dt, state, input_force, input_torques);   
        end
        
        % Evaluate NSS value at the current position and update personal best if needed
        function obj = evaluate_nss(obj, superpositionNSS, p_sources)
            obj.nss_value = superpositionNSS(obj.position, p_sources);
            if obj.nss_value > obj.nss_best_value
                obj.p_best = obj.position;
                obj.nss_best_value = obj.nss_value;
                %fprintf('Drone %d has received an NSS signal of  %.1f.\n', ...
                %    obj.identifier, obj.nss_best_value);
            end
            if obj.nss_value > 290000 && ~obj.victim_found_flag && isempty(obj.my_exclusion_zone)
                obj.victim_found_flag = true;
                % Remove inertia
                obj.inertia = 0.5;
                obj.max_velocity = 0.5; % ADDED NEW
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
            other_particle.shared_exclusion_zones = unique([other_particle.shared_exclusion_zones; obj.my_exclusion_zone], 'rows');
            % Display the sharing
            fprintf('Drone %d shared its exclusion zone: [%.1f, %.1f] with Drone %d\n', ...
                    obj.identifier, obj.my_exclusion_zone(1), obj.my_exclusion_zone(2), other_particle.identifier);
            fprintf('Drone %d now has the following shared exclusion zones: \n', other_particle.identifier);
            for i = 1:size(other_particle.shared_exclusion_zones, 1)
                fprintf('Zone %d centered in [%.1f, %.1f]\n', i, other_particle.shared_exclusion_zones(i, 1), other_particle.shared_exclusion_zones(i, 2));
            end
        end


        % Function to check if the drone is in any shared exclusion zone (shared by others)
        function [is_in_exclusion_zone, k] = check_if_in_exclusion_zone(obj)
            % Initialize the exclusion flag to false
            is_in_exclusion_zone = false;
            k = NaN;
            % Check if there are any shared exclusion zones
            if ~isempty(obj.shared_exclusion_zones)
                % Iterate through all shared exclusion zones
                for k = 1:size(obj.shared_exclusion_zones, 1)
                    exclusion_zone = obj.shared_exclusion_zones(k, :);  % Get exclusion zone position
                    distance_to_zone = norm(obj.position - exclusion_zone);  % Calculate distance to the zone
                    % Check if the drone is within the exclusion zone radius
                    if distance_to_zone < obj.exclusion_zone_radius
                        fprintf('Drone %d is in an exclusion zone.\n', obj.identifier);
                        is_in_exclusion_zone = true;  % Set the flag to true if inside an exclusion zone
                        break;  % Exit the loop once inside an exclusion zone
                    end
                end
            end
        end

        % Function to move the drone away from the exclusion zone
        function old_position = move_away_from_exclusion(obj, exclusion_zone, step_size)
            % Calculate the direction away from the exclusion zone
            direction_away = exclusion_zone - obj.position;
            direction_away_normalized = direction_away / norm(direction_away);  % Normalize the direction vector

            % Print drone's current position, exclusion zone, direction vector before and after normalization in one line
            fprintf('Drone Pos: [%.2f, %.2f], Exclusion Zone Pos: [%.2f, %.2f], Direction Vec (raw): [%.2f, %.2f], Direction Vec (norm): [%.2f, %.2f]\n', ...
                obj.position(1), obj.position(2), ...
                exclusion_zone(1), exclusion_zone(2), ...
                direction_away(1), direction_away(2), ...
                direction_away_normalized(1), direction_away_normalized(2));
            old_position = obj.position;
            % Update the position of the drone by moving it a small step away %IN OPPOSITE DIRECTION FROM WHERE YOU CAME FROM
            obj.position = obj.position + step_size * direction_away_normalized;
            % Print the new position of the drone
            fprintf('New Drone Position: [%.2f, %.2f]\n', obj.position(1), obj.position(2));
            %error("stop")
        end
    end
end

