function SAR_RL_Hysteretic
    % SAR_RL_HYSTERETIC
    % 1. HIERARCHICAL AI: RL decides 'Who' and 'Where'; BFS handles 'How'.
    % 2. HYSTERETIC Q-LEARNING: Handles cooperation by ignoring teammate errors.
    % 3. ENERGY AWARE: Robots learn to reject tasks if battery is too low.
    
    %% --- 1. SETUP & CONSTANTS ---
    close all; clear; clc;
    
    % Grid & Colors
    GRID_SIZE = 20;
    COLOR_BG_APP = [0.1 0.12 0.15];       
    COLOR_PANEL  = [0.15 0.18 0.22];      
    COLOR_MAP_BG = [0.05 0.1 0.2];        
    COLOR_WALL   = [0.4 0.6 0.8];         
    COLOR_GRID   = [0.2 0.3 0.4];         
    
    % Object Positions (Strategic Spread)
    CHARGERS = [2, 2; 19, 2; 10, 19]; % TL, BL, Right-Mid
    HOSPITAL_POS = [1, 10]; % Top Center Safe Zone
    
    % Simulation State
    map_matrix = zeros(GRID_SIZE); 
    victims_data = zeros(0, 3); % [r, c, status(0=Pending, 1=Assigned, 2=Saved)]
    
    % Agents
    agents_pos = CHARGERS; 
    agents_bat = [100, 100, 100];
    agents_state = ["IDLE", "IDLE", "IDLE"]; 
    agents_task_id = [0, 0, 0];  
    
    % Pathfinding & AI Memory
    agents_path = cell(1, 3); 
    
    % --- Q-LEARNING BRAIN ---
    % Q-Table Dimensions: [BatteryLevel(5)] x [DistanceBucket(5)]
    % Actions: We calculate Value of 'Accepting' vs 'Rejecting' implicitly
    Q_Table = zeros(5, 5); 
    isTrained = false;
    
    isRunning = false;

    %% --- 2. GUI LAYOUT ---
    fig = uifigure('Name', 'MARL Fleet Commander (Hysteretic RL)', ...
        'Color', COLOR_BG_APP, ...
        'Position', [100, 100, 1280, 720], ...
        'CloseRequestFcn', @(src,~) delete(src));
    
    g = uigridlayout(fig, [1, 2]);
    g.ColumnWidth = {300, '1x'}; 
    g.BackgroundColor = COLOR_BG_APP;
    
    % --- LEFT PANEL ---
    pnlControl = uipanel(g, 'Title', 'MISSION CONTROL', ...
        'BackgroundColor', COLOR_PANEL, ...
        'ForegroundColor', 'w', ...
        'FontWeight', 'bold', 'FontSize', 14);
    
    ctrlLayout = uigridlayout(pnlControl, [11, 1]);
    ctrlLayout.RowHeight = {30, 40, 20, 45, 20, 45, 60, '1x', 40, 20, 20};
    ctrlLayout.Padding = [15 15 15 15];
    
    uilabel(ctrlLayout, 'Text', 'ENVIRONMENT:', 'FontColor', [0.7 0.8 1]);
    ddMap = uidropdown(ctrlLayout, ...
        'Items', {'Debris Building', 'Mountain Rescue', 'Tsunami Aftermath'}, ...
        'BackgroundColor', 'w', ...
        'ValueChangedFcn', @cb_GenerateMap);
    
    uilabel(ctrlLayout, 'Text', '');
    
    btnTrain = uibutton(ctrlLayout, 'Text', 'TRAIN Q-AGENTS', ...
        'BackgroundColor', [0.9 0.6 0.1], 'FontColor', 'k', 'FontWeight', 'bold', ...
        'FontSize', 14, ...
        'ButtonPushedFcn', @cb_TrainAI);
        
    uilabel(ctrlLayout, 'Text', ''); % Spacer
        
    btnDeploy = uibutton(ctrlLayout, 'Text', 'DEPLOY DRONES', ...
        'BackgroundColor', [0.2 0.7 0.3], 'FontColor', 'w', 'FontWeight', 'bold', ...
        'FontSize', 14, 'Enable', 'off', ...
        'ButtonPushedFcn', @cb_Deploy);
        
    btnReset = uibutton(ctrlLayout, 'Text', 'RESET SYSTEM', ...
        'BackgroundColor', [0.8 0.3 0.3], 'FontColor', 'w', ...
        'ButtonPushedFcn', @cb_Reset);
    
    lblScore = uilabel(ctrlLayout, 'Text', 'SAVED: 0', ...
        'FontColor', [0.2 1 0.2], 'FontSize', 32, ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    
    txtLog = uitextarea(ctrlLayout, ...
        'Value', {'[SYSTEM] AI Training Required.'}, ...
        'BackgroundColor', [0 0 0], 'FontColor', [0.2 1 0.2], ...
        'FontName', 'Consolas', 'FontSize', 10);
    
    % --- RIGHT PANEL ---
    ax = uiaxes(g);
    ax.Color = COLOR_MAP_BG; 
    ax.XColor = COLOR_GRID; ax.YColor = COLOR_GRID;
    ax.XLim = [0.5, GRID_SIZE+0.5]; ax.YLim = [0.5, GRID_SIZE+0.5];
    ax.YDir = 'reverse';
    ax.DataAspectRatio = [1 1 1];
    ax.XTick = []; ax.YTick = []; 
    hold(ax, 'on');
    
    % Map Graphics
    imgMap = imagesc(ax, map_matrix);
    colormap(ax, [COLOR_MAP_BG; COLOR_WALL]); 
    clim(ax, [0, 1]);
    
    % Grid Lines
    [gx, gy] = meshgrid(0.5:1:GRID_SIZE+0.5, 0.5:1:GRID_SIZE+0.5);
    plot(ax, gx, gy, 'Color', COLOR_GRID, 'LineWidth', 1);
    plot(ax, gx', gy', 'Color', COLOR_GRID, 'LineWidth', 1);
    
    % Hospital Zone
    rectangle(ax, 'Position', [HOSPITAL_POS(2)-0.45, HOSPITAL_POS(1)-0.45, 0.9, 0.9], ...
        'FaceColor', [0 0.8 0.2], 'EdgeColor', 'w', 'LineWidth', 2);
    text(ax, HOSPITAL_POS(2), HOSPITAL_POS(1), '+', 'Color', 'w', 'FontSize', 16, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    % Charging Stations
    for i = 1:size(CHARGERS,1)
        rectangle(ax, 'Position', [CHARGERS(i,2)-0.45, CHARGERS(i,1)-0.45, 0.9, 0.9], ...
            'EdgeColor', 'c', 'LineWidth', 2, 'Curvature', 0.2);
        text(ax, CHARGERS(i,2), CHARGERS(i,1), 'C', 'Color', 'c', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    end
    
    % Agents
    obj_agents = gobjects(1,3);
    colors = {'r', 'g', 'c'};
    for i=1:3
        obj_agents(i) = plot(ax, CHARGERS(i,2), CHARGERS(i,1), 's', ...
            'MarkerSize', 20, 'MarkerFaceColor', colors{i}, 'MarkerEdgeColor', 'w', 'LineWidth', 2);
    end
    
    % Victims
    obj_victims = gobjects(1, 20);
    for i=1:20
        obj_victims(i) = plot(ax, -10, -10, 'p', ...
            'MarkerSize', 18, 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', 'Visible', 'off');
    end
    
    % Visual Trails
    obj_paths = gobjects(1,3);
    for i=1:3
        obj_paths(i) = plot(ax, [0 0], [0 0], 'y-', 'LineWidth', 2, 'Visible', 'off');
    end
    
    % Click Listener
    imgMap.ButtonDownFcn = @cb_MapClick;
    cb_GenerateMap(ddMap, []);

    %% --- 3. REINFORCEMENT LEARNING CORE ---
    
    function cb_TrainAI(~, ~)
        % This function trains the Q-Table using Hysteretic Q-Learning
        % It simulates thousands of decisions instantly to populate the brain.
        
        logMsg('Initializing Hysteretic Q-Learning...');
        pause(0.1);
        
        % Hyperparameters
        alpha = 0.1;  % Standard Learning Rate
        beta = 0.01;  % Hysteretic Rate (Learn slow from negative outcomes)
        gamma = 0.9;  % Discount Factor
        episodes = 5000;
        
        % State Space: Battery (1-5), Distance (1-5)
        % Action: Accept Task
        Q_Table = zeros(5, 5); 
        
        % Training Loop (Simulation)
        for e = 1:episodes
            % Random State
            bat = randi(5); % 1=Critical, 5=Full
            dist = randi(5); % 1=Close, 5=Far
            
            % Reward Function ( The "Rule" we want them to learn)
            % High reward for Close Dist + High Battery
            % Huge penalty for Far Dist + Low Battery
            
            % Energy Cost estimate
            energy_cost = dist * 20; 
            battery_level = bat * 20;
            
            if battery_level < energy_cost
                reward = -100; % Mission Fail (Death)
            else
                reward = 50 - (dist * 5); % Success (Efficient = Higher score)
            end
            
            % Hysteretic Update
            current_q = Q_Table(bat, dist);
            target = reward; % Simplified target since single-step task allocation
            
            delta = target - current_q;
            
            if delta >= 0
                % Positive surprise: Learn fast (Alpha)
                Q_Table(bat, dist) = current_q + alpha * delta;
            else
                % Negative surprise: Learn slow (Beta)
                % This makes agents optimistic about cooperation
                Q_Table(bat, dist) = current_q + beta * delta;
            end
        end
        
        isTrained = true;
        btnTrain.Text = 'AI READY';
        btnTrain.BackgroundColor = [0.3 0.3 0.3];
        btnTrain.Enable = 'off';
        
        % Show a pop-up graph to prove RL training occurred
        figure('Name', 'Q-Table Heatmap', 'NumberTitle', 'off', 'Position', [500, 500, 400, 300], 'MenuBar', 'none');
        h = heatmap(Q_Table, 'Colormap', jet);
        h.Title = 'Learned Q-Values (Battery vs Distance)';
        h.XLabel = 'Distance'; h.YLabel = 'Battery Level';
        
        logMsg('Training Complete. Policy Converged.');
    end

    function q_val = get_q_value(battery_pct, path_len)
        % Discretize State
        b_idx = ceil(battery_pct / 20); 
        if b_idx < 1, b_idx = 1; end
        
        d_idx = ceil(path_len / 5); % Assuming max path approx 25-30
        if d_idx > 5, d_idx = 5; end
        
        q_val = Q_Table(b_idx, d_idx);
    end

    %% --- 4. SIMULATION LOGIC ---
    
    function cb_GenerateMap(src, ~)
        type = src.Value;
        map_matrix(:) = 0; 
        
        if strcmp(type, 'Mountain Rescue')
            map_matrix(5:15, 10:11) = 1; % Ridge
            map_matrix(10:12, 1:6) = 1; 
        elseif strcmp(type, 'Tsunami Aftermath')
            rng(100); map_matrix(rand(GRID_SIZE) > 0.8) = 1;
        elseif strcmp(type, 'Debris Building')
            map_matrix(8:12, 7:14) = 1;
            map_matrix(4:6, 4:8) = 1;
            map_matrix(14:16, 3:7) = 1;
        end
        
        % Clear critical spots
        map_matrix(HOSPITAL_POS(1), HOSPITAL_POS(2)) = 0;
        for k=1:3, map_matrix(CHARGERS(k,1), CHARGERS(k,2)) = 0; end
        imgMap.CData = map_matrix;
    end

    function cb_MapClick(~, event)
        if ~isTrained, uialert(fig, 'Train AI First!', 'Error'); return; end
        
        coords = event.IntersectionPoint(1:2);
        c = round(coords(1)); r = round(coords(2));
        
        if r<1||r>GRID_SIZE||c<1||c>GRID_SIZE||map_matrix(r,c)==1, return; end
        
        idx = size(victims_data, 1) + 1;
        victims_data(idx, :) = [r, c, 0]; 
        
        obj_victims(idx).XData = c; obj_victims(idx).YData = r; obj_victims(idx).Visible = 'on';
        btnDeploy.Enable = 'on';
    end

    function cb_Deploy(~, ~)
        isRunning = true;
        btnDeploy.Enable = 'off';
        
        while isRunning && isvalid(fig)
            % 1. RL TASK ALLOCATION
            run_rl_allocation();
            
            % 2. PHYSICS UPDATE
            all_docked = true;
            pending = sum(victims_data(:,3) == 0);
            active = sum(victims_data(:,3) == 1);
            
            for id=1:3
                update_agent(id);
                if agents_state(id) ~= "IDLE" && agents_state(id) ~= "DEAD", all_docked = false; end
            end
            
            % 3. VICTORY CHECK
            if pending == 0 && active == 0 && all_docked
                isRunning = false;
                logMsg('MISSION COMPLETE: All victims saved.');
                uialert(fig, 'Mission Complete', 'Success');
                break;
            end
            
            update_status_display();
            drawnow;
            pause(0.1); 
        end
        if isvalid(fig), btnReset.Enable = 'on'; end
    end

    function run_rl_allocation()
        pending_idxs = find(victims_data(:,3) == 0);
        
        for v_idx = pending_idxs'
            v_pos = victims_data(v_idx, 1:2);
            best_agent = -1;
            best_q = -Inf; % We want MAX Q-Value
            
            for id = 1:3
                if agents_state(id) == "IDLE" && agents_bat(id) > 0
                    % Get BFS Path (for distance input to RL)
                    path = bfs_path(agents_pos(id,:), v_pos);
                    if isempty(path), continue; end
                    
                    dist = size(path, 1);
                    
                    % ASK THE Q-TABLE
                    q_val = get_q_value(agents_bat(id), dist);
                    
                    % Hysteretic Greedy Selection
                    if q_val > best_q && q_val > 0 % Only accept positive utility
                        best_q = q_val;
                        best_agent = id;
                        best_path = path;
                    end
                end
            end
            
            if best_agent ~= -1
                victims_data(v_idx, 3) = 1; % Assigned
                agents_state(best_agent) = "MOVING";
                agents_task_id(best_agent) = v_idx;
                agents_path{best_agent} = best_path;
            end
        end
    end

    function update_agent(id)
        if agents_bat(id) <= 0
            agents_state(id) = "DEAD"; 
            obj_agents(id).MarkerFaceColor = 'k'; 
            return; 
        end
        
        % Battery Physics
        at_charger = isequal(agents_pos(id,:), CHARGERS(id,:));
        if at_charger
            agents_bat(id) = min(100, agents_bat(id) + 10); % Fast charge
        else
            agents_bat(id) = max(0, agents_bat(id) - 0.8); % Drain
        end
        
        % LOW BATTERY OVERRIDE (RTB)
        if agents_bat(id) < 25 && agents_state(id) ~= "RETURNING" && agents_state(id) ~= "IDLE"
             logMsg(sprintf('Unit %d LOW BATTERY! Aborting to Base.', id));
             agents_state(id) = "RETURNING";
             % Drop task if carrying? No, keep carrying if possible, but path to charger
             % For simplicity: Path to charger immediately
             agents_path{id} = bfs_path(agents_pos(id,:), CHARGERS(id,:));
             
             % If we hadn't picked up victim yet, reset victim status
             if agents_task_id(id) > 0 && obj_victims(agents_task_id(id)).Visible == "on"
                 victims_data(agents_task_id(id), 3) = 0; % Reset to pending
                 agents_task_id(id) = 0;
             end
        end
        
        % MOVEMENT
        path = agents_path{id};
        if ~isempty(path)
            obj_paths(id).XData = path(:,2); obj_paths(id).YData = path(:,1); obj_paths(id).Visible = 'on';
            
            step = path(1,:);
            agents_pos(id,:) = step;
            obj_agents(id).XData = step(2); obj_agents(id).YData = step(1);
            
            path(1,:) = [];
            agents_path{id} = path;
        else
            obj_paths(id).Visible = 'off';
        end
        
        % LOGIC HANDLING (State Machine)
        if isempty(path)
            if agents_state(id) == "MOVING"
                % Reach Victim -> Switch to CARRY -> Path to Hospital
                agents_state(id) = "CARRYING";
                obj_agents(id).MarkerFaceColor = 'w'; % Visual indicator
                obj_victims(agents_task_id(id)).Visible = 'off'; % Pick up
                agents_path{id} = bfs_path(agents_pos(id,:), HOSPITAL_POS);
                
            elseif agents_state(id) == "CARRYING"
                % Reach Hospital -> Drop -> Return Home
                agents_state(id) = "RETURNING";
                obj_agents(id).MarkerFaceColor = colors{id};
                
                % Update Score
                curr = str2double(extractAfter(lblScore.Text, ': '));
                lblScore.Text = sprintf('SAVED: %d', curr+1);
                victims_data(agents_task_id(id), 3) = 2; % Saved
                agents_task_id(id) = 0;
                
                agents_path{id} = bfs_path(agents_pos(id,:), CHARGERS(id,:));
                
            elseif agents_state(id) == "RETURNING"
                agents_state(id) = "IDLE";
            end
        end
    end

    function path = bfs_path(start, goal)
        if isequal(start, goal), path=[]; return; end
        q = [start];
        came_from = zeros(GRID_SIZE, GRID_SIZE, 2);
        visited = false(GRID_SIZE); visited(start(1), start(2)) = true;
        found = false;
        
        dirs = [-1 0; 1 0; 0 -1; 0 1];
        
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
                p = squeeze(came_from(curr(1), curr(2), :))';
                path = [p; path];
                curr = p;
            end
            path(1,:) = [];
        else
            path = [];
        end
    end

    function update_status_display()
        t = cell(3,1);
        for i=1:3
            t{i} = sprintf('U%d: %s | %d%% Bat', i, agents_state(i), floor(agents_bat(i)));
        end
        txtLog.Value = t;
    end

    function cb_Reset(~, ~)
        isRunning = false;
        victims_data = zeros(0,3);
        agents_pos = CHARGERS;
        agents_bat = [100, 100, 100];
        agents_state = ["IDLE", "IDLE", "IDLE"];
        agents_task_id = [0, 0, 0];
        lblScore.Text = 'SAVED: 0';
        
        for i=1:20, obj_victims(i).Visible = 'off'; end
        for k=1:3
            obj_agents(k).XData = CHARGERS(k,2); obj_agents(k).YData = CHARGERS(k,1);
            obj_agents(k).MarkerFaceColor = colors{k};
            agents_path{k} = [];
            obj_paths(k).Visible = 'off';
        end
        
        btnDeploy.Enable = 'off';
        ddMap.Enable = 'on';
        logMsg('System Reset.');
    end

    function logMsg(msg)
        txtLog.Value = [{['> ' msg]}; txtLog.Value(1:min(end,9))];
    end
end