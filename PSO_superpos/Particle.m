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

classdef Particle
    properties
        position            % Current position of the particle (x, y)
        velocity            % Current velocity of the particle (x, y)
        p_best              % Best known position of the particle
        nss_best_value      % Best known NSS value at the personal best position
        nss_value           % Current NSS value at the particle's position
        bounds              % Boundaries of the search space
        group_idx           % Group index of the particle, indicating which group (source) the particle is assigned to
        identifier          % Unique identifier for the particle, used to distinguish it from other particles
        velocity_randomness % Factor controlling the amount of randomness added to the particle's velocity updates
        max_velocity        % Maximum allowed speed for the particle to prevent overly large velocity updates
        goal                % First goal they need to reach

    end
    
    methods
        % Constructor to initialize particle with position, velocity, and bounds
        function obj = Particle(goal, velocity_randomness, max_velocity, bounds, group_idx, identifier)
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
            fprintf('Drone initialized at pos: [%1d, %1d] and vel:[%.1f, %.1f]  with group index: %d\n', obj.position(1), obj.position(2), obj.velocity(1), obj.velocity(2),obj.group_idx);
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
            %fprintf('Particle %d NSS evaluated at: %.1d\n', obj.identifier, obj.nss_value);  % Debug print
        end
    end
end

