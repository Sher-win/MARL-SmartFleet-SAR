function SAR_Smart_Fleet_BFS
    % SAR_SMART_FLEET_BFS
    % 1. TRUE PATHFINDING: Uses BFS to navigate mazes without getting stuck.
    % 2. VISUAL PATHS: Draws yellow lines showing the planned route.
    % 3. SMART AUCTION: Efficiently assigns tasks based on real travel distance.
    
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
    CHARGERS = [1, 1; 20, 1; 10, 20];
    HOSPITAL_POS = [1, 10];
    
    % Simulation State
    map_matrix = zeros(GRID_SIZE); 
    victims_data = zeros(0, 3); % [r, c, status]
    
    % Agents
    agents_pos = CHARGERS; 
    agents_bat = [100, 100, 100];
    agents_state = ["IDLE", "IDLE", "IDLE"]; 
    agents_task_id = [0, 0, 0];  
    
    % Pathfinding Memory
    agents_path = cell(1, 3); % Stores the list of steps for each agent
    
    isRunning = false;
    isInitialized = false;

    %% --- 2. GUI LAYOUT ---
    fig = uifigure('Name', 'MARL Fleet Commander (BFS Navigation)', ...
        'Color', COLOR_BG_APP, ...
        'Position', [100, 100, 1280, 720], ...
        'CloseRequestFcn', @(src,~) delete(src));

    g = uigridlayout(fig, [1, 2]);
    g.ColumnWidth = {300, '1x'}; 
    g.BackgroundColor = COLOR_BG_APP;

    % --- LEFT PANEL ---
    pnlControl = uipanel(g, 'Title', 'FLEET COMMAND', ...
        'BackgroundColor', COLOR_PANEL, ...
        'ForegroundColor', 'w', ...
        'FontWeight', 'bold', 'FontSize', 14);
    
    ctrlLayout = uigridlayout(pnlControl, [10, 1]);
    ctrlLayout.RowHeight = {30, 40, 20, 45, 45, 60, '1x', 40, 20, 20};
    ctrlLayout.Padding = [15 15 15 15];

    uilabel(ctrlLayout, 'Text', 'ENVIRONMENT:', 'FontColor', [0.7 0.8 1]);
    ddMap = uidropdown(ctrlLayout, ...
        'Items', {'Open Field', 'Urban Obstacles', 'Debris Maze'}, ...
        'BackgroundColor', 'w', ...
        'ValueChangedFcn', @cb_GenerateMap);

    uilabel(ctrlLayout, 'Text', '');

    btnInit = uibutton(ctrlLayout, 'Text', 'INITIALIZE FLEET', ...
        'BackgroundColor', [0.2 0.5 0.9], 'FontColor', 'w', 'FontWeight', 'bold', ...
        'FontSize', 14, ...
        'ButtonPushedFcn', @cb_Initialize);

    btnDeploy = uibutton(ctrlLayout, 'Text', 'AUTO-DISPATCH', ...
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
        'Value', {'[SYSTEM] Fleet Offline.'}, ...
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

    % Graphics
    imgMap = imagesc(ax, map_matrix);
    colormap(ax, [COLOR_MAP_BG; COLOR_WALL]); 
    clim(ax, [0, 1]);

    % Grid
    [gx, gy] = meshgrid(0.5:1:GRID_SIZE+0.5, 0.5:1:GRID_SIZE+0.5);
    plot(ax, gx, gy, 'Color', COLOR_GRID, 'LineWidth', 1);
    plot(ax, gx', gy', 'Color', COLOR_GRID, 'LineWidth', 1);

    % Static Elements
    rectangle(ax, 'Position', [HOSPITAL_POS(2)-0.45, HOSPITAL_POS(1)-0.45, 0.9, 0.9], ...
        'FaceColor', [0 0.8 0.2], 'EdgeColor', 'w', 'LineWidth', 2);
    text(ax, HOSPITAL_POS(2), HOSPITAL_POS(1), 'H', 'Color', 'k', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    for i = 1:size(CHARGERS,1)
        rectangle(ax, 'Position', [CHARGERS(i,2)-0.45, CHARGERS(i,1)-0.45, 0.9, 0.9], ...
            'FaceColor', [0 0.8 1], 'EdgeColor', 'none', 'Curvature', 0.2);
        text(ax, CHARGERS(i,2), CHARGERS(i,1), '⚡', 'Color', 'k', 'HorizontalAlignment', 'center');
    end

    % Dynamic Objects
    obj_agents = gobjects(1,3);
    colors = {'r', 'g', 'c'};
    for i=1:3
        obj_agents(i) = plot(ax, CHARGERS(i,2), CHARGERS(i,1), 's', ...
            'MarkerSize', 22, 'MarkerFaceColor', colors{i}, 'MarkerEdgeColor', 'w', 'LineWidth', 2);
    end
    
    obj_victims = gobjects(1, 20);
    for i=1:20
        obj_victims(i) = plot(ax, -10, -10, 'p', ...
            'MarkerSize', 18, 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', 'Visible', 'off');
    end

    % Path Lines (Shows the BFS route)
    obj_paths = gobjects(1,3);
    for i=1:3
        obj_paths(i) = plot(ax, [0 0], [0 0], 'y-', 'LineWidth', 2, 'Visible', 'off');
    end

    imgMap.ButtonDownFcn = @cb_MapClick;
    cb_GenerateMap(ddMap, []);

    %% --- 3. LOGIC & INTELLIGENCE ---

    function cb_GenerateMap(src, ~)
        type = src.Value;
        map_matrix(:) = 0; 
        
        if strcmp(type, 'Urban Obstacles')
            map_matrix(4:6, 4:6) = 1; map_matrix(14:16, 14:16) = 1;
            map_matrix(4:6, 14:16) = 1; map_matrix(14:16, 4:6) = 1;
            map_matrix(9:11, 9:11) = 1;
            % Long wall to test pathfinding
            map_matrix(8, 2:15) = 1;
        elseif strcmp(type, 'Debris Maze')
            rng(42); noise = rand(GRID_SIZE);
            map_matrix(noise > 0.75) = 1;
        end
        
        % Ensure critical spots are open
        map_matrix(HOSPITAL_POS(1), HOSPITAL_POS(2)) = 0;
        for k=1:3, map_matrix(CHARGERS(k,1), CHARGERS(k,2)) = 0; end
        imgMap.CData = map_matrix;
    end

    function cb_Initialize(~, ~)
        logMsg('Calibrating Sensors...');
        pause(0.2);
        isInitialized = true;
        btnInit.Text = 'FLEET READY';
        btnInit.BackgroundColor = [0.2 0.7 0.3];
        logMsg('Fleet Online. Awaiting targets.');
        
        for k=1:3
            obj_agents(k).XData = CHARGERS(k,2);
            obj_agents(k).YData = CHARGERS(k,1);
            agents_path{k} = [];
        end
    end

    function cb_MapClick(~, event)
        if ~isInitialized
            uialert(fig, 'Initialize Fleet First!', 'Offline');
            return;
        end
        
        coords = event.IntersectionPoint(1:2);
        c = round(coords(1)); r = round(coords(2));
        
        if r<1 || r>GRID_SIZE || c<1 || c>GRID_SIZE, return; end
        if map_matrix(r,c) == 1, logMsg('Invalid Target (Wall).'); return; end
        
        % Add Victim
        idx = size(victims_data, 1) + 1;
        victims_data(idx, :) = [r, c, 0]; % 0 = Pending
        
        obj_victims(idx).XData = c;
        obj_victims(idx).YData = r;
        obj_victims(idx).Visible = 'on';
        
        logMsg(sprintf('Distress Signal: Sector [%d, %d]', r, c));
        btnDeploy.Enable = 'on';
    end

    function cb_Deploy(~, ~)
        isRunning = true;
        btnDeploy.Enable = 'off';
        btnReset.Enable = 'off';
        ddMap.Enable = 'off';
        
        while isRunning && isvalid(fig)
            % 1. Run Auction
            run_task_allocation();
            
            % 2. Update Agents
            all_docked = true;
            pending_count = sum(victims_data(:,3) == 0);
            active_count = sum(victims_data(:,3) == 1);
            
            for id=1:3
                update_agent_physics(id);
                if agents_state(id) ~= "IDLE"
                    all_docked = false;
                end
            end
            
            % Victory Condition
            if pending_count == 0 && active_count == 0 && all_docked
                isRunning = false;
                uialert(fig, 'All Victims Rescued!', 'Mission Success');
                break;
            end
            
            update_telemetry();
            drawnow;
            pause(0.15); % Smooth speed
        end
        
        if isvalid(fig)
            btnReset.Enable = 'on';
            ddMap.Enable = 'on';
        end
    end

    function run_task_allocation()
        pending_indices = find(victims_data(:,3) == 0);
        
        for i = 1:length(pending_indices)
            v_idx = pending_indices(i);
            v_pos = victims_data(v_idx, 1:2);
            
            best_agent = -1;
            min_path_len = Inf;
            best_path = [];
            
            % Auction: Bids based on REAL path length
            for id = 1:3
                if agents_state(id) == "IDLE" && agents_bat(id) > 20
                    % Calculate path to victim
                    path_to_v = bfs_path(agents_pos(id,:), v_pos);
                    
                    if ~isempty(path_to_v)
                        % Cost = Path Length
                        cost = size(path_to_v, 1);
                        if cost < min_path_len
                            min_path_len = cost;
                            best_agent = id;
                            best_path = path_to_v;
                        end
                    end
                end
            end
            
            % Assign Task
            if best_agent ~= -1
                victims_data(v_idx, 3) = 1; % Assigned
                agents_state(best_agent) = "MOVING";
                agents_task_id(best_agent) = v_idx;
                agents_path{best_agent} = best_path; % Assign the computed BFS path
                
                logMsg(sprintf('Unit %d accepted Mission %d (Path: %d steps)', ...
                    best_agent, v_idx, min_path_len));
            end
        end
    end

    function update_agent_physics(id)
        % Battery
        if isequal(agents_pos(id,:), CHARGERS(id,:))
            agents_bat(id) = min(100, agents_bat(id) + 5);
        else
            agents_bat(id) = max(0, agents_bat(id) - 0.5);
        end
        
        if agents_bat(id) <= 0
            agents_state(id) = "DEAD";
            obj_agents(id).MarkerFaceColor = [0.2 0.2 0.2];
            obj_paths(id).Visible = 'off';
            return;
        end
        
        % Follow Path
        path = agents_path{id};
        
        % Visualize Path
        if ~isempty(path)
            obj_paths(id).XData = path(:,2);
            obj_paths(id).YData = path(:,1);
            obj_paths(id).Visible = 'on';
            
            % Take 1 step
            next_step = path(1,:);
            agents_pos(id,:) = next_step;
            obj_agents(id).XData = next_step(2);
            obj_agents(id).YData = next_step(1);
            
            % Remove step from path
            path(1,:) = [];
            agents_path{id} = path;
        else
            obj_paths(id).Visible = 'off';
        end
        
        % Check Goals
        if isempty(path)
            if agents_state(id) == "MOVING"
                % Arrived at Victim
                agents_state(id) = "CARRYING";
                obj_agents(id).MarkerEdgeColor = 'y';
                
                % Hide Victim
                v_idx = agents_task_id(id);
                obj_victims(v_idx).Visible = 'off';
                
                % Calculate Return Path
                agents_path{id} = bfs_path(agents_pos(id,:), HOSPITAL_POS);
                
            elseif agents_state(id) == "CARRYING"
                % Arrived at Hospital
                agents_state(id) = "RETURNING";
                obj_agents(id).MarkerEdgeColor = 'w';
                
                % Update Score
                current = str2double(extractAfter(lblScore.Text, 'SAVED: '));
                lblScore.Text = sprintf('SAVED: %d', current + 1);
                victims_data(agents_task_id(id), 3) = 2; % Saved
                
                % Go Home
                agents_path{id} = bfs_path(agents_pos(id,:), CHARGERS(id,:));
                
            elseif agents_state(id) == "RETURNING"
                % Arrived Home
                agents_state(id) = "IDLE";
                agents_task_id(id) = 0;
            end
        end
    end

    function path = bfs_path(start_pos, goal_pos)
        % Standard BFS for Shortest Path on Grid
        if isequal(start_pos, goal_pos), path = []; return; end
        
        q = [start_pos];
        came_from = zeros(GRID_SIZE, GRID_SIZE, 2); % Stores parent pointers
        visited = false(GRID_SIZE);
        visited(start_pos(1), start_pos(2)) = true;
        
        found = false;
        
        while ~isempty(q)
            current = q(1,:);
            q(1,:) = [];
            
            if isequal(current, goal_pos)
                found = true;
                break;
            end
            
            % Neighbors (Up, Down, Left, Right)
            dirs = [-1 0; 1 0; 0 -1; 0 1];
            for k=1:4
                next = current + dirs(k,:);
                
                % Bounds Check
                if next(1)>0 && next(1)<=GRID_SIZE && next(2)>0 && next(2)<=GRID_SIZE
                    % Wall Check
                    if map_matrix(next(1), next(2)) == 0 && ~visited(next(1), next(2))
                        visited(next(1), next(2)) = true;
                        came_from(next(1), next(2), :) = current;
                        q = [q; next];
                    end
                end
            end
        end
        
        % Reconstruct Path
        if found
            path = goal_pos;
            curr = goal_pos;
            while ~isequal(curr, start_pos)
                par = squeeze(came_from(curr(1), curr(2), :))';
                path = [par; path];
                curr = par;
            end
            path(1,:) = []; % Remove start position
        else
            path = []; % No path found
        end
    end

    function update_telemetry()
        t = cell(4,1);
        for i=1:3
            stat = "OK";
            if agents_bat(i) < 30, stat = "LOW"; end
            taskStr = "IDLE";
            if agents_task_id(i) > 0, taskStr = sprintf("TASK #%d", agents_task_id(i)); end
            
            t{i} = sprintf('UNIT %d: %s | BAT: %d%% | %s', i, taskStr, floor(agents_bat(i)), stat);
        end
        t{4} = sprintf('QUEUE: %d Pending', sum(victims_data(:,3)==0));
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
            obj_agents(k).XData = CHARGERS(k,2);
            obj_agents(k).YData = CHARGERS(k,1);
            obj_agents(k).MarkerEdgeColor = 'w';
            obj_paths(k).Visible = 'off';
            agents_path{k} = [];
        end
        
        btnDeploy.Enable = 'off';
        ddMap.Enable = 'on';
        logMsg('System Reset.');
    end

    function logMsg(msg)
        current = txtLog.Value;
        txtLog.Value = [{['> ' msg]}; current(1:min(end,9))];
    end
end