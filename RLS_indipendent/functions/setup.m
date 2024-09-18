function [drones_list, artva, estimates] =  setup(drones_num, desired_goals)
    disp("Setup started!")
    global angles;
    global n_sources;
    global do_indipendent;

    % Set Artva info
    drones_list = cell([1, drones_num]);
    if do_indipendent
        artva = cell([1, n_sources]);
        estimates = cell([1, n_sources]);
        for i=1:n_sources
            artva{i} = Artva(desired_goals(i,:),i);
            estimates{i} = RLS(i, n_sources, do_indipendent);
        end
    elseif ~do_indipendent
        estimates = RLS(0, n_sources, do_indipendent); % 0 because we do not have an identifier we use the same algorithm for all sources
        artva = Artva(desired_goals(1,:), desired_goals(2,:));
    end

    % Set Drones info
    angles = zeros(1, drones_num);
    omega = 360/drones_num;
    for i = 1:drones_num
        drones_list{i} = Drone(i, [0, 0, 0]);
        omega_i = omega * (i-1);
        angles(i) = omega_i;
        m = tan(deg2rad(omega_i));
        if (omega_i > 315 && omega_i <= 360) || omega_i <= 45
            drones_list{i} = drones_list{i}.setGoal([100, m*100, 0]);
        elseif omega_i > 45 && omega_i <= 135
            drones_list{i} = drones_list{i}.setGoal([100/m, 100 0]);
        elseif omega_i > 135 && omega_i <= 225
            drones_list{i} = drones_list{i}.setGoal([-100, m*-100, 0]);
        elseif omega_i > 225 && omega_i <= 315
            drones_list{i} = drones_list{i}.setGoal([-100/m, -100, 0]);
        end
    end
    disp("Setup completed!")
    end