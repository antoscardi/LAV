classdef TotalMagneticField
    properties
        H_tot % Total magnetic field
    end
    
    methods
        % Constructor
        function obj = TotalMagneticField()
            obj.H_tot = [];
        end
        
        % Method to calculate the total magnetic field
        function obj = compute(obj, H1, H2)
            % Magnitudes of the two fields
            norm_H1 = norm(H1);
            norm_H2 = norm(H2);
            if norm(norm_H1) > eps && norm(norm_H2) > eps
                cos_alpha = dot(H1, H2) / (norm_H2 * norm_H2);
            else
                cos_alpha = eps;  
            end
            
            obj.H_tot = sqrt(norm_H1^2 + norm_H2^2 + 2 * norm_H1 * norm_H2 * cos_alpha);
        end
    end
end

