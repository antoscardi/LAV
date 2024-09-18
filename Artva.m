classdef Artva
    properties
        % Approximation values for a,b
        a = 1.291
        b = 1.028
        position
        position1
        position2
        identifier 
        SNR (1,1) double {mustBePositive} = 0.2 % Signal to Noise Ratio
        magnetic_field
        total_magnetic_field
    end
    
    methods
        % Constructor
        function obj = Artva(input1, input2)
            global do_indipendent
            if do_indipendent
                obj.position = input1;  
                obj.identifier = input2;  
            else
                obj.position1 = input1;  
                obj.position2 = input2;  
                obj.magnetic_field = MagneticField();
                obj.total_magnetic_field = TotalMagneticField();  
            end
        end

        % Method to compute the signal
        function [phi, signal] = getSignal(obj, dronePos, identifier)
            global do_indipendent
            global add_noise;
            global R_matrices;
            droneX = dronePos(1);
            droneY = dronePos(2);
            droneZ = dronePos(3);

            if do_indipendent
                artvaX = obj.position(1);
                artvaY = obj.position(2);
                artvaZ = obj.position(3);

                phi = [droneX^2;        ...
                   2 * droneX * droneY; ...
                   2 * droneX * droneZ; ...
                   droneY^2;            ...
                   2 * droneY * droneZ; ...
                   droneZ^2;            ...
                   -2 * droneX;         ...
                   -2 * droneY;         ...
                   -2 * droneZ;         ...
                   1];
                %{
                x = [(obj.b)^2; ...
                    0;         ...
                    0;         ...
                    (obj.a)^2; ...
                    0;         ...
                    (obj.a)^2; ...
                    artvaX;    ...
                    artvaY;    ...
                    artvaZ;    ...
                    obj.position * obj.position.']; % qui al centro andrebbe moltiplicato per M 
                    %}
                    M = R_matrices{identifier} * diag([obj.b, obj.a, obj.a]) * R_matrices{identifier}.';
                    %M = 3 * R_matrices{identifier} * [1; 0; 0;] * [0 1 0] * R_matrices{identifier}.';
                    x = [M(1,1); M(1,2); M(1,3); M(2,2); M(2,3); M(3,3); artvaX; artvaY; artvaZ; obj.position * M * obj.position.'];
                    signal = phi.' * x;
            
            elseif ~do_indipendent
                p = [droneX; droneY; droneZ];
                p1 = [obj.position1(1); obj.position1(2); obj.position1(3)];
                p2 = [obj.position2(1); obj.position2(2); obj.position2(3)];
                p1 = R_matrices{1}.' * (p - p1);
                p2 = R_matrices{2}.' * (p - p2);
                % First Magnetic Field
                %obj.magnetic_field = obj.magnetic_field.compute(p1);
                %H_1 = obj.magnetic_field.vector_field;
                % Second Magnetic Field
                %obj.magnetic_field = obj.magnetic_field.compute(p2);
                %H_2 = obj.magnetic_field.vector_field;
                % Total Magnetic Field
                %obj.total_magnetic_field = obj.total_magnetic_field.compute(H_1, H_2);
                %H_tot = obj.total_magnetic_field.H_tot;
                NSS1 = obj.NormalizedSourceStrength(p1);
                NSS2 = obj.NormalizedSourceStrength(p2);                
                signal = NSS1 + NSS2;
                phi = NaN;
            end

            if add_noise
                signal = obj.addNoise(signal);
            end 
        end

        % Method to add noise to the signal %%% METTERE DIPENDENZA DA DISTANZA
        function signalWithNoise = addNoise(obj, signal)
            amplitudeNoise = obj.SNR * signal;
            % a + (b-a)*rand() noise in an interval of [a,b] where a is the negative of the amplitude of the noise
            noise = -amplitudeNoise + 2 .* amplitudeNoise .* rand(size(signal));
            signalWithNoise = signal + noise;
        end

        function NSS = NormalizedSourceStrength(obj, p)
            % Calculate distance r between drone position (x, y) and the source
            x= p(1);
            y= p(2);
            r = sqrt(x^2 + y^2);
            % Prevent r from becoming too small (avoid division by near-zero)
            r_threshold = 1e-6;
            if r < r_threshold
                r = r_threshold;
            end
            
            % Calculate NSS (normalized source strength)
            NSS = 1/ r^4;
        end
        
    end
end
