classdef Artva
    properties
        % Approximation values for a,b
        a = 1.291
        b = 1.028
        position
        identifier 
        SNR (1,1) double {mustBePositive} = 0.2 % Signal to Noise Ratio
    end
    
    methods
        % Constructor
        function obj = Artva(p,i)
            if nargin == 2
                obj.position = p;
                obj.identifier = i;
            end
        end

        % Method to compute the signal
        function [phi, signal] = getSignal(obj, dronePos)
            global add_noise;
            droneX = dronePos(1);
            droneY = dronePos(2);
            droneZ = dronePos(3);

            artvaX = obj.position(1);
            artvaY = obj.position(2);
            artvaZ = obj.position(3);

            phi = [droneX^2;            ...
                   2 * droneX * droneY; ...
                   2 * droneX * droneZ; ...
                   droneY^2;            ...
                   2 * droneY * droneZ; ...
                   droneZ^2;            ...
                   -2 * droneX;         ...
                   -2 * droneY;         ...
                   -2 * droneZ;         ...
                   1];
            
            x = [(obj.b)^2; ...
                 0;         ...
                 0;         ...
                 (obj.a)^2; ...
                 0;         ...
                 (obj.a)^2; ...
                 artvaX;    ...
                 artvaY;    ...
                 artvaZ;    ...
                 obj.position * obj.position.'];

            signal = phi.' * x;
            disp(signal)

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
