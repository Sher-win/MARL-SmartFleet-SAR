function SAR_RL_Hysteretic
    % SAR_RL_HYSTERETIC_RESCUE_OPS
    % 1. HIERARCHICAL AI: RL decides 'Who' and 'Where'; BFS handles 'How'.
    % 2. MULTI-VICTIM CLUSTERING: Robots collect multiple nearby victims.
    % 3. AGENT RESCUE: Functional drones rescue dead drones to chargers.
    % 4. EXPLAINABILITY (XAI): Robots justify their decisions.
    
    %% --- 1. SETUP & CONSTANTS ---
    close all; clear; clc;
    
    % Grid & Colors
    GRID_SIZE = 20;
    COLOR_BG_APP = [0.1 0.12 0.15];       
    COLOR_PANEL  = [0.15 0.18 0.22];      
    COLOR_MAP_BG = [0.05 0.1 0.2];        
    COLOR_WALL   = [0.4 0.6 0.8];         
    COLOR_GRID   = [0.2 0.3 0.4];         
    
    % Object Positions
    CHARGERS = [2, 2; 19, 2; 10, 19]; % TL, BL, Right-Mid
    HOSPITAL_POS = [1, 10]; % Top Center
    
    % Simulation State
    map_matrix = zeros(GRID_SIZE); 
    victims_data = zeros(0, 3); 
    
    % Agents
    agents_pos = CHARGERS; 
    agents_bat = [100, 100, 100];
    % States: IDLE, MOVING, CARRYING_VICTIM, CARRYING_DRONE, RETURN_BASE, RETURN_CHARGER, CHARGING, DEAD, BEING_CARRIED
    agents_state = ["IDLE", "IDLE", "IDLE"]; 
    agents_task_id = [0, 0, 0];  
    agents_cargo = [0, 0, 0];    
    MAX_CAPACITY = 3;            
    
    % XAI Memory
    agent_logs = ["System Ready.", "System Ready.", "System Ready."];
    
    % Timer
    mission_tic = 0;
    mission_time = 0;
    
    % Pathfinding & AI Memory
    agents_path = cell(1, 3); 
    Q_Table = zeros(5, 5); 
    isTrained = false;
    isRunning = false;

    %% --- 2. GUI LAYOUT ---
    fig = uifigure('Name', 'XAI-MARL Fleet Commander (Drone Rescue)', ...
        'Color', COLOR_BG_APP, ...
        'Position', [50, 50, 1280, 750], ...
        'CloseRequestFcn', @(src,~) delete(src));
    
    g = uigridlayout(fig, [1, 2]);
    g.ColumnWidth = {350, '1x'}; 
    g.BackgroundColor = COLOR_BG_APP;
    
    % --- LEFT PANEL ---
    pnlControl = uipanel(g, 'Title', 'MISSION CONTROL', ...
        'BackgroundColor', COLOR_PANEL, ...
        'ForegroundColor', 'w', ...
        'FontWeight', 'bold', 'FontSize', 14);
    
    ctrlLayout = uigridlayout(pnlControl, [13, 1]); 
    ctrlLayout.RowHeight = {30, 40, 20, 45, 20, 45, 60, 40, 20, 50, 50, 50, 20};
    ctrlLayout.Padding = [10 10 10 10];
    
    uilabel(ctrlLayout, 'Text', 'ENVIRONMENT:', 'FontColor', [0.7 0.8 1]);
    ddMap = uidropdown(ctrlLayout, ...
        'Items', {'Debris Building', 'Mountain Rescue', 'Tsunami Aftermath'}, ...
        'BackgroundColor', 'w', ...
        'ValueChangedFcn', @cb_GenerateMap);
    
    uilabel(ctrlLayout, 'Text', '');
    
    btnTrain = uibutton(ctrlLayout, 'Text', 'TRAIN Q-AGENTS', ...
        'BackgroundColor', [0.9 0.6 0.1], 'FontColor', 'k', 'FontWeight', 'bold', ...
        'ButtonPushedFcn', @cb_TrainAI);
        
    uilabel(ctrlLayout, 'Text', ''); 
        
    btnDeploy = uibutton(ctrlLayout, 'Text', 'DEPLOY DRONES', ...
        'BackgroundColor', [0.2 0.7 0.3], 'FontColor', 'w', 'FontWeight', 'bold', ...
        'Enable', 'off', 'ButtonPushedFcn', @cb_Deploy);
        
    lblTimer = uilabel(ctrlLayout, 'Text', 'TIME: 00.00s', ...
        'FontColor', 'c', 'FontSize', 24, ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold', ...
        'BackgroundColor', [0.1 0.1 0.1]);
        
    btnReset = uibutton(ctrlLayout, 'Text', 'RESET SYSTEM', ...
        'BackgroundColor', [0.8 0.3 0.3], 'FontColor', 'w', ...
        'ButtonPushedFcn', @cb_Reset);
    
    lblScore = uilabel(ctrlLayout, 'Text', 'SAVED: 0', ...
        'FontColor', [0.2 1 0.2], 'FontSize', 32, ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold');
        
    logA = uitextarea(ctrlLayout, 'Value', agent_logs(1), 'BackgroundColor', [0.2 0 0], 'FontColor', 'w', 'FontSize', 10);
    logB = uitextarea(ctrlLayout, 'Value', agent_logs(2), 'BackgroundColor', [0 0.2 0], 'FontColor', 'w', 'FontSize', 10);
    logC = uitextarea(ctrlLayout, 'Value', agent_logs(3), 'BackgroundColor', [0 0 0.2], 'FontColor', 'w', 'FontSize', 10);
    
    % --- RIGHT PANEL ---
    ax = uiaxes(g);
    ax.Color = COLOR_MAP_BG; 
    ax.XColor = COLOR_GRID; ax.YColor = COLOR_GRID;
    ax.XLim = [0.5, GRID_SIZE+0.5]; ax.YLim = [0.5, GRID_SIZE+0.5];
    ax.YDir = 'reverse';
    ax.DataAspectRatio = [1 1 1];
    ax.XTick = []; ax.YTick = []; 
    hold(ax, 'on');
    
    imgMap = imagesc(ax, map_matrix);
    colormap(ax, [COLOR_MAP_BG; COLOR_WALL]); clim(ax, [0, 1]);
    
    [gx, gy] = meshgrid(0.5:1:GRID_SIZE+0.5, 0.5:1:GRID_SIZE+0.5);
    plot(ax, gx, gy, 'Color', COLOR_GRID); plot(ax, gx', gy', 'Color', COLOR_GRID);
    
    rectangle(ax, 'Position', [HOSPITAL_POS(2)-0.45, HOSPITAL_POS(1)-0.45, 0.9, 0.9], ...
        'FaceColor', [0 0.8 0.2], 'EdgeColor', 'w', 'LineWidth', 2);
    text(ax, HOSPITAL_POS(2), HOSPITAL_POS(1), '+', 'Color', 'w', 'FontSize', 16, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    for i = 1:size(CHARGERS,1)
        rectangle(ax, 'Position', [CHARGERS(i,2)-0.45, CHARGERS(i,1)-0.45, 0.9, 0.9], ...
            'EdgeColor', 'c', 'LineWidth', 2, 'Curvature', 0.2);
        text(ax, CHARGERS(i,2), CHARGERS(i,1), 'C', 'Color', 'c', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    end
    
    obj_agents = gobjects(1,3); colors = {'r', 'g', 'c'};
    for i=1:3, obj_agents(i) = plot(ax, CHARGERS(i,2), CHARGERS(i,1), 's', ...
            'MarkerSize', 20, 'MarkerFaceColor', colors{i}, 'MarkerEdgeColor', 'w', 'LineWidth', 2); end
    
    obj_victims = gobjects(1, 20);
    for i=1:20, obj_victims(i) = plot(ax, -10, -10, 'p', ...
            'MarkerSize', 18, 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', 'Visible', 'off'); end
    
    obj_paths = gobjects(1,3);
    for i=1:3, obj_paths(i) = plot(ax, [0 0], [0 0], 'y-', 'LineWidth', 2, 'Visible', 'off'); end
    
    imgMap.ButtonDownFcn = @cb_MapClick;
    cb_GenerateMap(ddMap, []);

    %% --- 3. REINFORCEMENT LEARNING CORE ---
    function cb_TrainAI(~, ~)
        logA.Value = "Training Q-Policy...";
        pause(0.1);
        alpha = 0.1; beta = 0.01; gamma = 0.9; episodes = 5000;
        Q_Table = zeros(5, 5); 
        for e = 1:episodes
            bat = randi(5); dist = randi(5);
            energy_cost = dist * 20; battery_level = bat * 20;
            if battery_level < energy_cost, reward = -100; else, reward = 50 - (dist * 5); end
            curr = Q_Table(bat, dist); delta = reward - curr;
            if delta >= 0, Q_Table(bat, dist) = curr + alpha * delta;
            else, Q_Table(bat, dist) = curr + beta * delta; end
        end
        isTrained = true;
        btnTrain.Text = 'AI READY';
        btnTrain.Enable = 'off';
        logA.Value = "Training Complete.";
    end

    function q_val = get_q_value(battery_pct, path_len)
        b_idx = ceil(battery_pct / 20); if b_idx < 1, b_idx = 1; end
        d_idx = ceil(path_len / 5); if d_idx > 5, d_idx = 5; end
        q_val = Q_Table(b_idx, d_idx);
    end

    %% --- 4. SIMULATION LOGIC & XAI ---
    function cb_GenerateMap(src, ~)
        type = src.Value; map_matrix(:) = 0; 
        if strcmp(type, 'Mountain Rescue'), map_matrix(5:15, 10:11)=1; map_matrix(10:12, 1:6)=1; 
        elseif strcmp(type, 'Tsunami Aftermath'), rng(100); map_matrix(rand(GRID_SIZE)>0.8)=1;
        elseif strcmp(type, 'Debris Building'), map_matrix(8:12, 7:14)=1; map_matrix(4:6, 4:8)=1; map_matrix(14:16, 3:7)=1; end
        map_matrix(HOSPITAL_POS(1), HOSPITAL_POS(2)) = 0;
        for k=1:3, map_matrix(CHARGERS(k,1), CHARGERS(k,2)) = 0; end
        imgMap.CData = map_matrix;
    end

    function cb_MapClick(~, event)
        if ~isTrained, uialert(fig, 'Train AI First!', 'Error'); return; end
        coords = event.IntersectionPoint(1:2); c = round(coords(1)); r = round(coords(2));
        if r<1||r>GRID_SIZE||c<1||c>GRID_SIZE||map_matrix(r,c)==1, return; end
        idx = size(victims_data, 1) + 1;
        victims_data(idx, :) = [r, c, 0]; 
        obj_victims(idx).XData = c; obj_victims(idx).YData = r; obj_victims(idx).Visible = 'on';
        btnDeploy.Enable = 'on';
    end

    function cb_Deploy(~, ~)
        isRunning = true; btnDeploy.Enable = 'off'; mission_tic = tic;
        
        while isRunning && isvalid(fig)
            % 1. TASK ALLOCATION & RESCUE LOGIC
            run_rl_allocation();
            
            % 2. PHYSICS
            all_docked = true;
            pending_victims = sum(victims_data(:,3) == 0);
            active_victims = sum(victims_data(:,3) == 1);
            
            dead_drones = 0;
            for id=1:3
                if agents_state(id) == "DEAD", dead_drones = dead_drones + 1; end
            end
            
            for id=1:3
                update_agent(id);
                if agents_state(id) ~= "IDLE" && agents_state(id) ~= "DEAD" && agents_state(id) ~= "BEING_CARRIED"
                    all_docked = false; 
                end
            end
            
            mission_time = toc(mission_tic);
            lblTimer.Text = sprintf('TIME: %.2fs', mission_time);
            
            % Update Logs
            logA.Value = agent_logs(1); logB.Value = agent_logs(2); logC.Value = agent_logs(3);
            
            % Victory Condition: No victims left, no dead drones left on field
            dead_on_field = 0;
            for id=1:3
                if agents_state(id) == "DEAD", dead_on_field = 1; end
            end
            
            if pending_victims == 0 && active_victims == 0 && all_docked && dead_on_field == 0
                isRunning = false;
                uialert(fig, sprintf('Mission Complete! Time: %.2fs', mission_time), 'Success');
                break;
            end
            
            % Fail Safe: If everyone is dead or out of battery
            alive_count = 0;
            for id=1:3, if agents_bat(id) > 0, alive_count = alive_count + 1; end; end
            if alive_count == 0
                 isRunning = false;
                 uialert(fig, 'Mission Failed: All Units Lost.', 'Failure');
                 break;
            end
            
            drawnow; pause(0.1); 
        end
        if isvalid(fig), btnReset.Enable = 'on'; end
    end

    function run_rl_allocation()
        % Check for DEAD DRONES first (Highest Priority)
        dead_drone_id = -1;
        for id=1:3
            if agents_state(id) == "DEAD"
                % Check if someone is already rescuing this drone
                is_being_rescued = false;
                for rescuer=1:3
                    if agents_task_id(rescuer) == -id, is_being_rescued = true; end
                end
                if ~is_being_rescued
                    dead_drone_id = id;
                    break; % Handle one dead drone at a time
                end
            end
        end
        
        for id = 1:3
            % Skip if I am dead or being carried
            if agents_state(id) == "DEAD" || agents_state(id) == "BEING_CARRIED", continue; end
            
            % If I am healthy and there is a dead drone, I should help
            if dead_drone_id ~= -1 && dead_drone_id ~= id
                % Only intervene if I am not already carrying a victim
                if agents_cargo(id) == 0 && (agents_state(id) == "IDLE" || agents_state(id) == "MOVING")
                    agents_state(id) = "CARRYING_DRONE"; % Flag intention to rescue drone
                    agents_task_id(id) = -dead_drone_id; % Negative ID indicates drone rescue
                    agent_logs(id) = sprintf("MAYDAY RECEIVED: Unit %d down.\nAction: Initiating Rescue.", dead_drone_id);
                    % Once assigned, clear the flag so others don't swarm
                    dead_drone_id = -1; 
                    continue; 
                end
            end
            
            % Normal Victim Logic
            if (agents_state(id) == "IDLE" || agents_state(id) == "MOVING") && agents_cargo(id) < MAX_CAPACITY
                best_q = -Inf; best_v_idx = -1; best_path = [];
                pending_idxs = find(victims_data(:,3) == 0);
                
                for v_idx = pending_idxs'
                    v_pos = victims_data(v_idx, 1:2);
                    path = bfs_path(agents_pos(id,:), v_pos);
                    if isempty(path), continue; end
                    dist = size(path, 1);
                    
                    clustering_bonus = 0; if dist < 5, clustering_bonus = 50; end
                    q_val = get_q_value(agents_bat(id), dist) + clustering_bonus;
                    
                    if q_val > best_q && q_val > 0
                        best_q = q_val; best_v_idx = v_idx; best_path = path;
                    end
                end
                
                if best_v_idx ~= -1 && agents_state(id) == "IDLE"
                    victims_data(best_v_idx, 3) = 1; 
                    agents_state(id) = "MOVING";
                    agents_task_id(id) = best_v_idx;
                    agents_path{id} = best_path;
                    agent_logs(id) = sprintf("Unit %d: Target Victim #%d.\nQ-Val: %.1f.", id, best_v_idx, best_q);
                elseif agents_cargo(id) > 0 && agents_state(id) == "IDLE"
                    agents_state(id) = "CARRYING_VICTIM";
                    agents_path{id} = bfs_path(agents_pos(id,:), HOSPITAL_POS);
                    agent_logs(id) = "Capacity Limit. Delivering victims.";
                end
            end
        end
    end

    function update_agent(id)
        if agents_state(id) == "BEING_CARRIED"
            % Visual updates happen via the carrier
            agent_logs(id) = "CRITICAL: Being transported to charger.";
            return;
        end
        
        if agents_bat(id) <= 0
            if agents_state(id) ~= "DEAD"
                agents_state(id) = "DEAD"; obj_agents(id).MarkerFaceColor = 'k';
                agent_logs(id) = "CRITICAL FAILURE: Battery Depleted.";
            end
            return; 
        end
        
        % Battery Physics
        if isequal(agents_pos(id,:), CHARGERS(id,:))
            agents_bat(id) = min(100, agents_bat(id) + 10);
        else
            agents_bat(id) = max(0, agents_bat(id) - 0.8);
        end
        
        % LOW BATTERY INTERRUPT (Only if not rescuing a drone)
        if agents_bat(id) < 25 && ~contains(agents_state(id), ["RETURN", "IDLE", "CARRY", "DEAD", "BEING"])
             agents_state(id) = "RETURN_CHARGER";
             agents_path{id} = bfs_path(agents_pos(id,:), CHARGERS(id,:));
             agent_logs(id) = "LOW BATTERY. Returning to Base.";
             if agents_task_id(id) > 0 && victims_data(agents_task_id(id), 3) == 1
                 victims_data(agents_task_id(id), 3) = 0; agents_task_id(id) = 0;
             end
        end
        
        % Movement Step
        path = agents_path{id};
        if ~isempty(path)
            obj_paths(id).XData = path(:,2); obj_paths(id).YData = path(:,1); obj_paths(id).Visible = 'on';
            step = path(1,:); agents_pos(id,:) = step;
            obj_agents(id).XData = step(2); obj_agents(id).YData = step(1);
            path(1,:) = []; agents_path{id} = path;
            
            % If carrying a drone, move the dead drone with me
            if agents_state(id) == "RETURN_CHARGER" && agents_task_id(id) < 0
                dead_id = abs(agents_task_id(id));
                agents_pos(dead_id,:) = step;
                obj_agents(dead_id).XData = step(2); obj_agents(dead_id).YData = step(1);
            end
        else
            obj_paths(id).Visible = 'off';
        end
        
        % State Transitions
        if isempty(path)
            state = agents_state(id);
            
            if state == "MOVING"
                v_idx = agents_task_id(id);
                obj_victims(v_idx).Visible = 'off'; victims_data(v_idx, 3) = 2; 
                agents_cargo(id) = agents_cargo(id) + 1; agents_task_id(id) = 0; 
                agent_logs(id) = "Victim Secured. Scanning...";
                
                if agents_cargo(id) >= MAX_CAPACITY
                    agents_state(id) = "CARRYING_VICTIM";
                    agents_path{id} = bfs_path(agents_pos(id,:), HOSPITAL_POS);
                else
                    agents_state(id) = "IDLE"; 
                end
                
            elseif state == "CARRYING_VICTIM"
                count = agents_cargo(id); agents_cargo(id) = 0;
                curr = str2double(extractAfter(lblScore.Text, ': '));
                lblScore.Text = sprintf('SAVED: %d', curr + count);
                agents_state(id) = "RETURN_CHARGER";
                agents_path{id} = bfs_path(agents_pos(id,:), CHARGERS(id,:));
                agent_logs(id) = "Victims delivered. RTB.";
                
            elseif state == "CARRYING_DRONE"
                % Arrived at dead drone location
                dead_id = abs(agents_task_id(id));
                agents_state(dead_id) = "BEING_CARRIED";
                
                % Switch to Return mode carrying the drone
                agents_state(id) = "RETURN_CHARGER";
                % Return to MY charger (or nearest, we assume home base for simplicity)
                agents_path{id} = bfs_path(agents_pos(id,:), CHARGERS(id,:));
                agent_logs(id) = sprintf("Unit %d secured. Returning to charger.", dead_id);
                
            elseif state == "RETURN_CHARGER"
                % If I was carrying a drone, drop it
                if agents_task_id(id) < 0
                    dead_id = abs(agents_task_id(id));
                    agents_state(dead_id) = "IDLE"; % Revived!
                    agents_bat(dead_id) = 50; % Jumpstart
                    agents_task_id(id) = 0;
                    agent_logs(id) = sprintf("Unit %d Revived. Mission resuming.", dead_id);
                    agent_logs(dead_id) = "System Rebooted. Rejoining swarm.";
                end
                agents_state(id) = "IDLE";
            end
        end
        
        % Visuals
        if agents_cargo(id) > 0, obj_agents(id).MarkerFaceColor = 'w';
        elseif agents_task_id(id) < 0, obj_agents(id).MarkerFaceColor = 'y'; % Rescuing Drone
        elseif agents_bat(id) > 0, obj_agents(id).MarkerFaceColor = colors{id}; end
    end

    function path = bfs_path(start, goal)
        if isequal(start, goal), path=[]; return; end
        q = [start]; came_from = zeros(GRID_SIZE, GRID_SIZE, 2);
        visited = false(GRID_SIZE); visited(start(1), start(2)) = true;
        found = false; dirs = [-1 0; 1 0; 0 -1; 0 1];
        while ~isempty(q)
            curr = q(1,:); q(1,:) = [];
            if isequal(curr, goal), found = true; break; end
            for k=1:4
                next = curr + dirs(k,:);
                if next(1)>0 && next(1)<=GRID_SIZE && next(2)>0 && next(2)<=GRID_SIZE
                    if map_matrix(next(1), next(2)) == 0 && ~visited(next(1), next(2))
                        visited(next(1), next(2)) = true;
                        came_from(next(1), next(2), :) = curr;
                        q = [q; next];
                    end
                end
            end
        end
        if found
            path = goal; curr = goal;
            while ~isequal(curr, start)
                p = squeeze(came_from(curr(1), curr(2), :))'; path = [p; path]; curr = p;
            end
            path(1,:) = [];
        else, path = []; end
    end

    function cb_Reset(~, ~)
        isRunning = false; victims_data = zeros(0,3);
        agents_pos = CHARGERS; agents_bat = [100, 100, 100];
        agents_state = ["IDLE", "IDLE", "IDLE"]; agents_task_id = [0, 0, 0]; agents_cargo = [0, 0, 0];
        lblScore.Text = 'SAVED: 0'; lblTimer.Text = 'TIME: 00.00s';
        agent_logs = ["Reset.", "Reset.", "Reset."];
        logA.Value=""; logB.Value=""; logC.Value="";
        for i=1:20, obj_victims(i).Visible = 'off'; end
        for k=1:3
            obj_agents(k).XData = CHARGERS(k,2); obj_agents(k).YData = CHARGERS(k,1);
            obj_agents(k).MarkerFaceColor = colors{k}; agents_path{k} = []; obj_paths(k).Visible = 'off';
        end
        btnDeploy.Enable = 'off'; ddMap.Enable = 'on';
    end
end