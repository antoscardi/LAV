classdef RLS < handle
    properties
        drones_num
        est_H
        est_Y
        est_S
        est_beta
        est_X
        identifier
        estimated_position
    end
    methods 
        % Constructor
        function obj = RLS(drones_num, identifier)
            if nargin == 2
                obj.drones_num = drones_num;
		        obj.est_H = zeros(10, drones_num);
		        obj.est_Y = zeros(drones_num, 1);
                obj.est_S = eye(10);
                obj.est_beta = 0.999;
                obj.est_X = zeros(10, 1);
                obj.estimated_position = zeros(3, 1);
                obj.identifier = identifier;
            end
        end
        % Estimation
        function estimate = rls(obj, drones_list, artva)
            for i = 1:obj.drones_num           
                [phi, signal] = artva.getSignal(drones_list{i}.position);
                obj.est_H(:,i) = phi;
                obj.est_Y(i) = signal;
            end
            obj.est_X = obj.est_X + pinv(obj.est_S)*obj.est_H*(obj.est_Y - obj.est_H.'*obj.est_X);
            obj.est_S = obj.est_beta*obj.est_S + obj.est_H * obj.est_H.';
            obj.estimated_position = [obj.est_X(7), obj.est_X(8), obj.est_X(9)];
            estimate = obj.estimated_position(1:2);
        end
    end
end 