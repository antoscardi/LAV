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
                %disp(p1)
                p1 = R_matrices{1}.' * (p - p1);
                p2 = R_matrices{2}.' * (p - p2);
                %disp(p1)
                [~, H_1] = obj.magnetic_field.calculateFieldAtPoints(p1);
                [~, H_2] = obj.magnetic_field.calculateFieldAtPoints(p2);
                %disp(H_1)
                %disp(H_2)
                cos_alpha = dot(H_1, H_2)/(norm(H_1) * norm(H_2));
                %disp(rad2deg(cos_alpha))
                %obj.a = (obj.a)^2/(0.629 + 0.449*cos_alpha)^(3/2);
                %obj.b = (obj.b)^2/(0.629 + 0.449*cos_alpha)^(3/2);
                %disp(obj.a)
                %disp(obj.b)

                phi = [ -2 * droneX; -2 * droneY; -2 * droneZ];
                phi_known = [droneX^2; 2 * droneX * droneY; 2 * droneX * droneZ; droneY^2; 2 * droneY * droneZ; droneZ^2; 1];
                % First signal
                M1 = R_matrices{1} * diag([2, -1, -1]) * R_matrices{1}.';
                M2 = R_matrices{2} * diag([2, -1, -1]) * R_matrices{2}.';
                %x1_known = [M1(1,1); M1(1,2); M1(1,3); M1(2,2); M1(2,3); M1(3,3); obj.position1 * M1 * obj.position1.'];
                %x2_known = [M2(1,1); M2(1,2); M2(1,3); M2(2,2); M2(2,3); M2(3,3); obj.position2 * M2 * obj.position2.'];
                %x1 = [M1(1,1); M1(1,2); M1(1,3); M1(2,2); M1(2,3); M1(3,3); obj.position1(1); obj.position1(2); obj.position1(3); obj.position1 * M1 * obj.position1.'];
                %x2 = [M2(1,1); M2(1,2); M2(1,3); M2(2,2); M2(2,3); M2(3,3); obj.position2(1); obj.position2(2); obj.position2(3); obj.position2 * M2 * obj.position2.'];
                %x1 = [(obj.b)^2; 0; 0; (obj.a)^2; 0; (obj.a)^2; obj.position1(1); obj.position1(2); obj.position1(3); obj.position1 * obj.position1.'];
                %x2 = [(obj.b)^2; 0; 0; (obj.a)^2; 0; (obj.a)^2; obj.position2(1); obj.position2(2); obj.position2(3); obj.position2 * obj.position2.'];
                x1 = [obj.position1(1); obj.position1(2); obj.position1(3);];
                x2 = [obj.position2(1); obj.position2(2); obj.position2(3);]; 
                signal1 = phi.' * x1 + phi.' * x2;

                % Second signal
                %x1 = [(obj.b)^2; 0; 0; (obj.a)^2; 0; (obj.a)^2; obj.position1(1); obj.position1(2); obj.position1(3); obj.position1 * obj.position1.'];
                %x2 = [(obj.b)^2; 0; 0; (obj.a)^2; 0; (obj.a)^2; obj.position2(1); obj.position2(2); obj.position2(3); obj.position2 * obj.position2.'];
                B1 = 3 * R_matrices{1} * [1; 0; 0;] * [0 1 0] * R_matrices{1}.';
                B2 = 3 * R_matrices{2} * [1; 0; 0;] * [0 1 0] * R_matrices{2}.';
                x1_known = [B1(1,1); B1(1,2); B1(1,3); B1(2,2); B1(2,3); B1(3,3); obj.position1 * M1 * obj.position1.'];
                x2_known = [B2(1,1); B2(1,2); B2(1,3); B2(2,2); B2(2,3); B2(3,3); obj.position2 * M2 * obj.position2.'];
                %x1 = [B1(1,1); B1(1,2); B1(1,3); B1(2,2); B1(2,3); B1(3,3); obj.position1(1); obj.position1(2); obj.position1(3); obj.position1 * B1 * obj.position1.'];
                %x2 = [B2(1,1); B2(1,2); B2(1,3); B2(2,2); B2(2,3); B2(3,3); obj.position2(1); obj.position2(2); obj.position2(3); obj.position2 * B2 * obj.position2.'];
                signal2 = phi.' * x1 / norm(p1)^5 + phi.' * x2 /norm(p2)^5 - phi_known.' * (x1_known + x2_known);

                % Third signal
                %C1 = 3 * R_matrices{1} * [1; 0; 0;] * [0 0 1] * R_matrices{1}.';
                %C2 = 3 * R_matrices{2} * [1; 0; 0;] * [0 0 1] * R_matrices{2}.';
                %x1 = [C1(1,1); C1(1,2); C1(1,3); C1(2,2); C1(2,3); C1(3,3); obj.position1(1); obj.position1(2); obj.position1(3); obj.position1 * C1 * obj.position1.'];
                %x2 = [C2(1,1); C2(1,2); C2(1,3); C2(2,2); C2(2,3); C2(3,3); obj.position2(1); obj.position2(2); obj.position2(3); obj.position2 * C2 * obj.position2.'];
                signal3 = phi.' * x1 + phi.' * x2 + cos_alpha * (phi.' * (x1 + x2)) ;

                signal = [signal1 signal2 signal3];
                %disp(signal)
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
    end
end
