% Global optimization algorithm through Differential Evolution (DE)
classdef DE
    properties
        popSize;    % Population size
        F;          % Mutation factor
        CR;         % Crossover probability
        maxGen;     % Maximum number of generations per timestep
        lowerBound; % Lower bound of search space
        upperBound; % Upper bound of search space
        dim;        % Dimension of the problem (3 * n for n sources)
        objValues;  % Precomputed objective function values (array)
    end
    
    methods
        function obj = DE(popSize, F, CR, maxGen, lowerBound, upperBound, dim, objValues)
            % Constructor to initialize the DE parameters
            obj.popSize = popSize;
            obj.F = F;
            obj.CR = CR;
            obj.maxGen = maxGen;
            obj.lowerBound = lowerBound;
            obj.upperBound = upperBound;
            obj.dim = dim;
            obj.objValues = objValues; % Store precomputed objective function values
        end
        
        function optimizeUsingPrecomputedValues(obj)
            % Initialize population
            pop = obj.lowerBound + rand(obj.popSize, obj.dim) .* (obj.upperBound - obj.lowerBound);
            fitness = arrayfun(@(i) obj.objValues(1), 1:obj.popSize)';
            
            numTimesteps = length(obj.objValues);
            
            for t = 1:numTimesteps
                disp(['Timestep ', num2str(t)]);
                
                % Use precomputed objective function value for the current timestep
                currentObjectiveValue = obj.objValues(t);
                
                % Evolve population for a fixed number of generations
                for gen = 1:obj.maxGen
                    for i = 1:obj.popSize
                        % Mutation
                        idxs = randperm(obj.popSize, 3);
                        while any(idxs == i)
                            idxs = randperm(obj.popSize, 3);
                        end
                        x1 = pop(idxs(1), :);
                        x2 = pop(idxs(2), :);
                        x3 = pop(idxs(3), :);

                        mutant = x1 + obj.F * (x2 - x3);
                        mutant = max(min(mutant, obj.upperBound), obj.lowerBound); % Ensure within bounds

                        % Crossover
                        trial = pop(i, :);
                        jRand = randi(obj.dim);
                        for j = 1:obj.dim
                            if rand() < obj.CR || j == jRand
                                trial(j) = mutant(j);
                            end
                        end

                        % Selection
                        % Instead of calculating the objective function, use precomputed value
                        % Assuming that each trial has a separate precomputed value associated
                        % Here, we simulate that the current trial's fitness is the same as the precomputed
                        trialFitness = currentObjectiveValue; 
                        if trialFitness < fitness(i)
                            pop(i, :) = trial;
                            fitness(i) = trialFitness;
                        end
                    end
                end
                
                % Record the best solution for the current timestep
                [bestVal, bestIdx] = min(fitness);
                bestPos = pop(bestIdx, :);
                
                % Display best fitness and position of current timestep
                disp(['Best Fitness at Timestep ', num2str(t), ' = ', num2str(bestVal)]);
                disp(['Best Position: ', num2str(bestPos)]);
                
                % Pause to simulate real-time processing
                pause(0.1); % Adjust pause duration as needed
            end
        end
    end
end


