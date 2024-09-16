classdef RLS < handle
    properties
        est_H
        est_Y
        est_S
        est_beta
        est_X
        est_P
        identifier
        estimated_position % COULD BE MORE THAN ONE
        magnetic_field
        K
        true_Y
        J
        P
    end
    methods 
        % Constructor
        function obj = RLS(identifier, n_sources, do_indipendent)
            global drones_num
            obj.identifier = identifier;
            if ~do_indipendent
                obj.est_beta = 0.97;
                obj.est_P = eye(n_sources*3);
                obj.est_X = zeros(n_sources*3, 1);
                obj.est_Y = zeros(drones_num, 1);
                obj.true_Y = zeros(drones_num, 1);
                obj.J = zeros(drones_num, 3);
                obj.magnetic_field = MagneticField();
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
        % Estimation
        function estimate = rls(obj, drones_list, artva)
            global do_indipendent
            global n_sources
            global drones_num
            
            % Perturbation step size for numerical differentiation
            h = 1e-4;
            
            if ~do_indipendent
                % Retrieve signals from drones
                for i = 1:drones_num           
                    [~, signal] = artva.getSignal(drones_list{i}.position, obj.identifier);
                    obj.true_Y(i) = signal;
                    drone_pos = drones_list{i}.position';
                    p_local = 
                    disp(obj.est_X)
                    obj.est_Y(i) = obj.magnetic_field.compute();
                    
                end
                
                % Compute the error between the predicted and actual signals
                e_t = obj.true_Y - obj.est_Y;
                
                for j = 1:3*n_sources
                    theta_perturbed = obj.est_X;
                    theta_perturbed(j) = theta_perturbed(j) + h;
                    % Perturbed signal prediction
                    y_perturbed = signal_model(theta_perturbed, drone_positions);
                    % Numerical derivative (finite difference)
                    obj.J(:, j) = (y_perturbed - obj.est_Y) / h;
                end
                
                % Update the Kalman gain
                obj.K = obj.est_P * obj.J' / (obj.est_beta * eye(drones_num) + obj.J * obj.est_P * obj.J');
                % Update the estimated position of the source
                obj.est_X = obj.est_X + obj.K * e_t;
                % Update the covariance matrix
                obj.est_P = (obj.est_P - obj.K * obj.J * obj.est_P) / obj.est_beta;  
                % Return the updated parameter estimates
                estimate = obj.est_X;
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