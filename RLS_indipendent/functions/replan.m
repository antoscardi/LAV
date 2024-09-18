function new_drones_list = replan(drones_list, drones_num, est_artva_pos)
    global trajectory_type;
    global dronesSetted;
    global distributed_estimation_mode;
    if trajectory_type == "rect"
        for i = 1:drones_num
            if(drones_list{i}.state ~= "idle" || ~drones_list{i}.isAtGoal())
                new_drones_list = drones_list;
                return
            end
        end
        disp("Replanning!")
        dronesSetted = true;
        new_drones_list = cell([1, drones_num]);
        for i = 1:drones_num
            current_drone = drones_list{i};
            g = current_drone.goal;
            if(g(2) ~= 0)
                current_drone = current_drone.setGoal([g(1), 0, g(3)]);
            else
                current_drone = current_drone.setGoal([g(1), 100, g(3)]);
            end
            new_drones_list{i} = current_drone;
            clear current_drone g;
        end
    
    elseif trajectory_type == "circ"
        all_idle = true;
        dronesSetted = true;
    
        for i = 1:drones_num
            if drones_list{i}.state ~= "idle" || ~drones_list{i}.isAtGoal()
                all_idle = false;
            end
        end
    
        if all_idle
            disp("Replanning!");
            new_drones_list = cell([1, drones_num]);
            for i = 1:drones_num
                current_drone = drones_list{i};
                if(~distributed_estimation_mode)
                    disp(est_artva_pos)
                    current_drone = current_drone.setGoal(est_artva_pos);
                else
                    new_objective = est_artva_pos(:,i)';
                    current_drone = current_drone.setGoal(new_objective);
                end
                new_drones_list{i} = current_drone;
            end
        else
            new_drones_list = drones_list;
        end
        return
    
    elseif trajectory_type == "patrol"
        dronesSetted = true;
        new_drones_list = drones_list;
        isAllIdle = true;
        firstIsIdle = true;
        lastIsIdle = true;
        
        % check whether central drones are idle or not
        for i = 2:drones_num-1
            if (drones_list{i}.state ~= "idle" || ~drones_list{i}.isAtGoal())
                isAllIdle = false;
            end
        end
        
        % check if first drone is at the bottom right corner (idle)
        if (drones_list{1}.state ~= "idle" || ~drones_list{1}.isAtGoal())
            firstIsIdle = false;
        end
        
        % check if last drone is at the top left corner (idle)
        if (drones_list{drones_num}.state ~= "idle" || ~drones_list{drones_num}.isAtGoal())
            lastIsIdle = false;
        end
        
        % if the first and the last drone are in position (position specified
        % before) move to the top right angle (common goal)
        if (firstIsIdle && lastIsIdle)
            disp("Replanning");
            current_drone = drones_list{1};
            current_drone = current_drone.setGoal([100,100,0]);
            new_drones_list{1} = current_drone;
            clear current_drone;
    
            current_drone = drones_list{drones_num};
            current_drone = current_drone.setGoal([100,100,0]);
            new_drones_list{drones_num} = current_drone;
            clear current_drone;
        end
    
        % if the central drones are all idle move towards the estimated position of
        % the artva
        if isAllIdle
            disp("Replanning!");
            for i =1:drones_num
                if (i ~= 1 && i ~= drones_num)
                    current_drone = drones_list{i};
                    if(~distributed_estimation_mode)
                        current_drone = current_drone.setGoal(est_artva_pos);
                    else
                        new_objective = est_artva_pos(:,i)';
                        current_drone = current_drone.setGoal(new_objective);
                    end
                    new_drones_list{i} = current_drone;
                end
            end
        end
    
        return
    else
        error("Unknown trajectory type: %s", trajectory);
    end
    end
    