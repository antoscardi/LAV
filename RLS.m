classdef RLS < handle
    properties
        est_H
        est_Y
        est_S
        est_beta
        est_X
        identifier
        estimated_position % COULD BE MORE THAN ONE
    end
    methods 
        % Constructor
        function obj = RLS(identifier, n_sources, do_indipendent)
            global drones_num
            obj.identifier = identifier;
            if ~do_indipendent
                obj.est_H = zeros(6 , drones_num);
                obj.est_Y = zeros(drones_num, 3);
                obj.est_S = eye(6, 6);
                obj.est_beta = 0.95;
                obj.est_X = zeros(6, 1);
            elseif do_indipendent
                obj.est_H = zeros(10, drones_num);
                obj.est_Y = zeros(drones_num, 1);
                obj.est_S = eye(10);
                obj.est_beta = 0.999;
                obj.est_X = zeros(10, 1);
                %obj.estimated_position = zeros(3, 1);
            end
        end
        
        % Estimation
        function estimate = rls(obj, drones_list, artva)
            global do_indipendent
            global n_sources
            global drones_num
            if ~do_indipendent
                %  Y RIMANE DI DIMENSIONE 10 PERCHE ogni drone rimane 
                for i = 1:drones_num           
                    [phi, signal] = artva.getSignal(drones_list{i}.position, obj.identifier);
                    % Repeat phi for each signal (assuming 3 signals)
                    phi_stacked = [phi; phi];
                    %disp(size(phi_stacked))
                    obj.est_H(:, i) = phi_stacked;
                    obj.est_Y(i, :) = signal;
                end
                
                % Compute the combined error for all signals
                combined_error = zeros(size(obj.est_X));  % Initialize combined error as a 20x1 vector
                for col = 1:3
                    error = obj.est_Y(:, col) - obj.est_H.' * obj.est_X;  % Error for each signal
                    combined_error = combined_error + obj.est_H * error;  % Aggregate the error
                end

                % Perform the RLS update by minimizing the combined error
                obj.est_X = obj.est_X + pinv(obj.est_S) * combined_error;
                obj.est_S = obj.est_beta*obj.est_S + obj.est_H * obj.est_H.';

                estimate1 = [obj.est_X(1), obj.est_X(2), obj.est_X(3)];
                estimate2 = [obj.est_X(4), obj.est_X(5), obj.est_X(6)];
                estimate = [estimate1(1), estimate2(1);...
                            estimate1(2), estimate2(2)];
                disp(estimate)

            elseif do_indipendent
                for i = 1:drones_num           
                    [phi, signal] = artva.getSignal(drones_list{i}.position, obj.identifier);
                    obj.est_H(:,i) = phi;
                    obj.est_Y(i) = signal;
                end
                obj.est_X = obj.est_X + pinv(obj.est_S)*obj.est_H*(obj.est_Y - obj.est_H.'*obj.est_X);
                obj.est_S = obj.est_beta*obj.est_S + obj.est_H * obj.est_H.';
                obj.estimated_position = [obj.est_X(7), obj.est_X(8), obj.est_X(9)];
                estimate = [obj.estimated_position(1);...
                            obj.estimated_position(2)];
            end
        end
    end
end 