%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                   ARTVAs CLASS                                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The ARTVAs class handles the calculation of NSS (Normalized Source Strength) signals from multiple sources. It 
% provides methods to compute the combined NSS value from all sources and individual NSS values based on the 
% distance between a drone and a source.
%
% Methods:
% - superpositionNSS: Calculates the total NSS received by a drone from all sources.
% - send_signal_NSS: Calculates the NSS (signal strength) from a specific source based on distance.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef ARTVAs
    properties
        C % Scaling factor for NSS calculation
    end
    
    methods
        % Constructor to initialize scaling factor C
        function obj = ARTVAs(C)
            if nargin < 1
                obj.C = 100000000;  % It was 100 before
            else
                obj.C = C;
            end
        end

        % Superposition NSS Function: Sum of the NSS from all sources
        function value = superpositionNSS(obj, position, p_sources)
            value = 0;
            for i = 1:size(p_sources, 1)
                value = value + obj.send_signal_NSS(position, p_sources(i, :));
            end
            return;
        end
        

        % NSS Function: Normalized Source Strength (Signal Function)
        function NSS_value = send_signal_NSS(obj, position, p_source)
            % Calculate the distance between the drone and the source
            r = sqrt(sum((position - p_source).^2));  % Use element-wise power
            
            % Prevent division by zero by setting a minimum distance
            r = max(r, 1e-5);
            
            % Calculate normalized source strength (NSS) based on distance
            NSS_value = obj.C / r^4;  % Signal strength diminishes rapidly with distance
            %fprintf('NSS calculated at distance: %.1d, NSS value: %.1d\n', r, NSS_value);  % Debug print
        end
    end
end


