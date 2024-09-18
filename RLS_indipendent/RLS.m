classdef RLS < handle
    properties
        est_H, est_Y, est_S, est_beta, est_X, est_P, identifier, estimated_position, K, true_Y, J, P, Q, R
    end
    methods
        % Constructor
        function obj = RLS(identifier, n_sources, do_indipendent)
            global drones_num
            obj.identifier = identifier;
            if ~do_indipendent
                obj.est_beta = 0.95;
                obj.est_H = zeros(3*n_sources, drones_num);
                obj.est_Y = zeros(drones_num, 1);
                obj.est_S = eye(3*n_sources);
                obj.est_X = zeros(3*n_sources, 1); % State vector (unknown positions)
                obj.est_P = eye(3 * n_sources); % Use a smaller initial covariance matrix
                obj.Q = 1e-2 * eye(3 * n_sources);  % Increase process noise covariance
                obj.R = 1e-5 * eye(drones_num);     % Further decrease measurement noise covariance                             
                obj.estimated_position = zeros(6, 1);
            elseif do_indipendent
                obj.est_H = zeros(10, drones_num);
                obj.est_Y = zeros(drones_num, 1);
                obj.est_S = eye(10);
                obj.est_beta = 0.999;
                obj.est_X = zeros(10, 1);
            end
        end
        
        % Estimation
        function estimate = rls(obj, drones_list, artva)
            global do_indipendent, 
            global n_sources, 
            global drones_num
            if ~do_indipendent
                for i = 1:drones_num
                    [~, signal] = artva.getSignal(drones_list{i}.position, obj.identifier);
                    obj.est_Y(i) = signal;
                end
                X_pred = obj.est_X; P_pred = obj.est_P + obj.Q;
                if any(isnan(P_pred(:))) || any(isinf(P_pred(:))), error('P_pred has NaN or Inf'); end
                H_k = obj.computeJacobianH(X_pred, drones_list);
                if any(isnan(H_k(:))) || any(isinf(H_k(:))), error('H_k has NaN or Inf'); end
                S = H_k * P_pred * H_k.' + obj.R + 1e-6 * eye(drones_num); % Add regularization to prevent singularity
                if rcond(S) < 1e-15, disp('S is poorly conditioned'); end
                K_k = P_pred * H_k.' * pinv(S);
                disp('Kalman Gain:'), disp(K_k);
                Y_pred = obj.h(drones_list, X_pred);
                residual = obj.est_Y - Y_pred;
                residual = residual * 10; % Scale residual to increase updates
                disp('Residual:'), disp(residual);
                if any(isnan(residual)) || any(isinf(residual)), error('Residual has NaN or Inf'); end
                damping_factor = 0; 
                obj.est_X = X_pred + damping_factor * (K_k * residual);
                if any(isnan(obj.est_X)) || any(isinf(obj.est_X)), error('State update resulted in NaN or Inf'); end
                obj.est_P = (eye(size(K_k,1)) - K_k * H_k) * P_pred;
                obj.estimated_position = [obj.est_X(1), obj.est_X(2), obj.est_X(3), obj.est_X(4), obj.est_X(5), obj.est_X(6)];
                disp('Updated estimates:'), disp(obj.estimated_position);
                estimate = [obj.est_X(1), obj.est_X(2); ...
                            obj.est_X(4), obj.est_X(5)];
            elseif do_indipendent
                for i = 1:drones_num
                    [phi, signal] = artva.getSignal(drones_list{i}.position, obj.identifier);
                    obj.est_H(:,i) = phi;
                    obj.est_Y(i) = signal;
                end
                obj.est_X = obj.est_X + pinv(obj.est_S) * obj.est_H * (obj.est_Y - obj.est_H.' * obj.est_X);
                obj.est_S = obj.est_beta * obj.est_S + obj.est_H * obj.est_H.';
                obj.estimated_position = [obj.est_X(7), obj.est_X(8), obj.est_X(9)];
                estimate = [obj.estimated_position(1); ...
                            obj.estimated_position(2)];
            end
        end

        % Jacobian of observation model (H matrix)
        function H = computeJacobianH(obj, X, drones_list)
            global drones_num
            H = zeros(drones_num, 6); % 6 unknowns: [x1, y1, z1, x2, y2, z2]
            epsilon = 1e-6; % Small threshold to prevent division by very small numbers
            for i = 1:drones_num
                p_r = drones_list{i}.position';
                p1 = p_r - X(1:3); p2 = p_r - X(4:6);
                norm_p1 = max(norm(p1), epsilon); norm_p2 = max(norm(p2), epsilon);
                H(i, 1:3) = p1 / norm_p1^3; % Derivative w.r.t. [x1, y1, z1]
                H(i, 4:6) = p2 / norm_p2^3; % Derivative w.r.t. [x2, y2, z2]
            end
        end

        % Nonlinear observation model
        function Y = h(obj, drones_list, estimated_sources)
            global drones_num, 
            global R_matrices
            Y = zeros(drones_num, 1);
            for i = 1:drones_num
                p_r = drones_list{i}.position';
                ps1 = [estimated_sources(1); estimated_sources(2); estimated_sources(3)];
                ps2 = [estimated_sources(4); estimated_sources(5); estimated_sources(6)];
                p1 = R_matrices{1}.' * (p_r - ps1); p2 = R_matrices{2}.' * (p_r - ps2);
                NSS1 = obj.NormalizedSourceStrength(p1); NSS2 = obj.NormalizedSourceStrength(p2);
                NSS_tot = NSS1 + NSS2;
                Y(i) = NSS_tot;
            end
        end

        % Normalized source strength function
        function NSS = NormalizedSourceStrength(obj, p)
            x = p(1); y = p(2); r = sqrt(x^2 + y^2);
            r_threshold = 1e-6; if r < r_threshold, r = r_threshold; end
            NSS = 1 / r^4;
        end
    end
end

