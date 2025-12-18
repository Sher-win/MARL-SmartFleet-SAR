function SAR_RL_Hysteretic
    % SAR_RL_HYSTERETIC_UI_FIX
    % 1. LAYOUT REPAIR: Fixed "missing middle box" bug by defining exact rows.
    % 2. SPACE OPTIMIZATION: Combined Timer & Score to free up space.
    % 3. LOGIC: All RL + Pathfinding features preserved.
    
    %% --- 1. SETUP & CONSTANTS ---
    close all; clear; clc;
    
    % --- CYBER-TACTICAL THEME ---
    GRID_SIZE = 20;
    COLOR_BG_APP = [0.08 0.09 0.11];       
    COLOR_PANEL  = [0.13 0.15 0.18];       
    COLOR_MAP_BG = [0.02 0.02 0.03];       
    COLOR_GRID   = [0.2 0.25 0.3];         
    COLOR_WALL   = [0.3 0.35 0.4];         
    
    % Neon Accents
    C_RED   = [1.0 0.3 0.3];
    C_GRN   = [0.2 0.9 0.4];
    C_BLU   = [0.3 0.6 1.0];
    C_GOLD  = [1.0 0.8 0.0];
    C_CYAN  = [0.0 1.0 1.0];
    
    % Object Positions
    CHARGERS = [2, 2; 19, 2; 10, 19]; 
    HOSPITAL_POS = [1, 10]; 
    
    % Simulation State
    map_matrix = zeros(GRID_SIZE); 
    victims_data = zeros(0, 3); 
    
    % Agents
    agents_pos = CHARGERS; 
    agents_bat = [100, 100, 100];
    agents_state = ["IDLE", "IDLE", "IDLE"]; 
    agents_task_id = [0, 0, 0];  
    agents_cargo = [0, 0, 0];    
    MAX_CAPACITY = 3;            
    
    % XAI History
    agent_logs = { {'[SYSTEM] Online.'}, {'[SYSTEM] Online.'}, {'[SYSTEM] Online.'} };
    
    % Timer
    mission_tic = 0; mission_time = 0;
    
    % AI Memory
    agents_path = cell(1, 3); 
    Q_Table = zeros(5, 5); 
    isTrained = false; isRunning = false;

    %% --- 2. GUI LAYOUT ---
    fig = uifigure('Name', 'MARL COMMAND: TACTICAL OPS', ...
        'Color', COLOR_BG_APP, ...
        'Position', [50, 50, 1200, 700], ... 
        'CloseRequestFcn', @(src,~) delete(src));
    
    g = uigridlayout(fig, [1, 2]);
    g.ColumnWidth = {380, '1x'}; 
    g.BackgroundColor = COLOR_BG_APP;
    
    % --- LEFT PANEL (MISSION CONTROL) ---
    pnlControl = uipanel(g, 'Title', '  MISSION OPERATIONS  ', ...
        'BackgroundColor', COLOR_PANEL, ...
        'ForegroundColor', [0.8 0.8 0.9], ...
        'FontWeight', 'bold', 'FontSize', 14, ...
        'FontName', 'Roboto');
    
    % STRICT ROW DEFINITION (Prevents Squishing)
    % 1. Env Label (25)
    % 2. Dropdown (30)
    % 3. Train Btn (40)
    % 4. Deploy Btn (40)
    % 5. Reset Btn (40)
    % 6. Stats Row (Timer/Score) (40)
    % 7. Header A (20)
    % 8. Log A (1x)
    % 9. Header B (20)
    % 10. Log B (1x)
    % 11. Header C (20)
    % 12. Log C (1x)
    ctrlLayout = uigridlayout(pnlControl, [12, 1]); 
    ctrlLayout.RowHeight = {25, 30, 40, 40, 40, 40, 20, '1x', 20, '1x', 20, '1x'}; 
    ctrlLayout.Padding = [10 10 10 10];
    ctrlLayout.RowSpacing = 5;
    
    % 1. Environment
    uilabel(ctrlLayout, 'Text', 'SECTOR SELECTION:', 'FontColor', [0.6 0.7 0.8], 'FontWeight', 'bold');
    
    % 2. Dropdown
    ddMap = uidropdown(ctrlLayout, ...
        'Items', {'Debris Building', 'Mountain Rescue', 'Tsunami Aftermath'}, ...
        'BackgroundColor', [0.2 0.2 0.25], 'FontColor', 'w', ...
        'ValueChangedFcn', @cb_GenerateMap);
    
    % 3. Train
    btnTrain = uibutton(ctrlLayout, 'Text', 'INITIALIZE NEURAL POLICY', ...
        'BackgroundColor', [0.8 0.5 0.0], 'FontColor', 'w', 'FontWeight', 'bold', ...
        'FontSize', 12, 'ButtonPushedFcn', @cb_TrainAI);
        
    % 4. Deploy
    btnDeploy = uibutton(ctrlLayout, 'Text', 'INITIATE DEPLOYMENT', ...
        'BackgroundColor', [0.1 0.6 0.3], 'FontColor', 'w', 'FontWeight', 'bold', ...
        'FontSize', 14, 'Enable', 'off', 'ButtonPushedFcn', @cb_Deploy);
        
    % 5. Reset
    btnReset = uibutton(ctrlLayout, 'Text', 'ABORT / RESET', ...
        'BackgroundColor', [0.7 0.2 0.2], 'FontColor', 'w', 'FontWeight', 'bold', ...
        'ButtonPushedFcn', @cb_Reset);
    
    % 6. STATS ROW (Combined Timer + Score)
    statsGrid = uigridlayout(ctrlLayout, [1, 2]);
    statsGrid.Padding = [0 0 0 0];
    statsGrid.BackgroundColor = COLOR_PANEL;
    
    lblTimer = uilabel(statsGrid, 'Text', 'T+ 00.0s', ...
        'FontColor', C_CYAN, 'FontSize', 18, 'FontName', 'Consolas', ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    
    lblScore = uilabel(statsGrid, 'Text', 'SAVED: 0', ...
        'FontColor', C_GRN, 'FontSize', 18, ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold');
        
    % --- XAI LOGS (Explicit Rows Assigned) ---
    % 7. Header A
    uilabel(ctrlLayout, 'Text', 'UNIT ALPHA (RED) LOGS:', 'FontColor', C_RED, 'FontWeight', 'bold', 'FontSize', 10);
    % 8. Log A
    logA = uilistbox(ctrlLayout, 'Items', agent_logs{1}, ...
        'BackgroundColor', [0.1 0.05 0.05], 'FontColor', [1 0.8 0.8], ...
        'FontName', 'Consolas', 'FontSize', 10); 
    
    % 9. Header B
    uilabel(ctrlLayout, 'Text', 'UNIT BRAVO (GRN) LOGS:', 'FontColor', C_GRN, 'FontWeight', 'bold', 'FontSize', 10);
    % 10. Log B
    logB = uilistbox(ctrlLayout, 'Items', agent_logs{2}, ...
        'BackgroundColor', [0.05 0.1 0.05], 'FontColor', [0.8 1 0.8], ...
        'FontName', 'Consolas', 'FontSize', 10);
    
    % 11. Header C
    uilabel(ctrlLayout, 'Text', 'UNIT CHARLIE (BLU) LOGS:', 'FontColor', C_BLU, 'FontWeight', 'bold', 'FontSize', 10);
    % 12. Log C
    logC = uilistbox(ctrlLayout, 'Items', agent_logs{3}, ...
        'BackgroundColor', [0.05 0.05 0.1], 'FontColor', [0.8 0.9 1], ...
        'FontName', 'Consolas', 'FontSize', 10);
    
    % --- RIGHT PANEL (TACTICAL MAP) ---
    ax = uiaxes(g);
    ax.Color = COLOR_MAP_BG; 
    ax.XColor = [0.3 0.3 0.3]; ax.YColor = [0.3 0.3 0.3];
    ax.XLim = [0.5, GRID_SIZE+0.5]; ax.YLim = [0.5, GRID_SIZE+0.5];
    ax.YDir = 'reverse';
    ax.DataAspectRatio = [1 1 1];
    ax.XTick = []; ax.YTick = []; 
    hold(ax, 'on');
    
    % Map Graphics
    imgMap = imagesc(ax, map_matrix);
    colormap(ax, [COLOR_MAP_BG; COLOR_WALL]); clim(ax, [0, 1]);
    
    [gx, gy] = meshgrid(0.5:1:GRID_SIZE+0.5, 0.5:1:GRID_SIZE+0.5);
    plot(ax, gx, gy, 'Color', COLOR_GRID, 'LineWidth', 0.5); 
    plot(ax, gx', gy', 'Color', COLOR_GRID, 'LineWidth', 0.5);
    
    rectangle(ax, 'Position', [HOSPITAL_POS(2)-0.45, HOSPITAL_POS(1)-0.45, 0.9, 0.9], ...
        'FaceColor', [0 0.3 0], 'EdgeColor', C_GRN, 'LineWidth', 2);
    plot(ax, HOSPITAL_POS(2), HOSPITAL_POS(1), 'p', 'MarkerFaceColor', C_GRN, 'MarkerEdgeColor', 'w', 'MarkerSize', 24);
    text(ax, HOSPITAL_POS(2), HOSPITAL_POS(1)+1.2, 'SAFE ZONE', 'Color', C_GRN, 'FontSize', 10, 'HorizontalAlignment', 'center', 'FontName', 'Consolas');
    
    for i = 1:size(CHARGERS,1)
        rectangle(ax, 'Position', [CHARGERS(i,2)-0.45, CHARGERS(i,1)-0.45, 0.9, 0.9], ...
            'EdgeColor', C_CYAN, 'LineWidth', 2, 'Curvature', 0.2, 'LineStyle', '--');
        text(ax, CHARGERS(i,2), CHARGERS(i,1), '⚡', 'Color', C_CYAN, 'FontSize', 14, 'HorizontalAlignment', 'center');
    end
    
    obj_agents = gobjects(1,3); 
    agent_colors = {C_RED, C_GRN, C_BLU};
    for i=1:3
        plot(ax, CHARGERS(i,2), CHARGERS(i,1), 'd', 'MarkerSize', 25, 'MarkerFaceColor', agent_colors{i}, 'MarkerEdgeColor', 'none', 'PickableParts', 'none');
        obj_agents(i) = plot(ax, CHARGERS(i,2), CHARGERS(i,1), 'd', 'MarkerSize', 12, 'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'k', 'LineWidth', 1);
    end
    
    obj_victims = gobjects(1, 20);
    for i=1:20
        obj_victims(i) = plot(ax, -10, -10, 'h', 'MarkerSize', 16, 'MarkerFaceColor', C_GOLD, 'MarkerEdgeColor', [1 0.5 0], 'LineWidth', 1.5, 'Visible', 'off');
    end
    
    obj_paths = gobjects(1,3);
    for i=1:3, obj_paths(i) = plot(ax, [0 0], [0 0], '-', 'Color', [agent_colors{i} 0.6], 'LineWidth', 2, 'Visible', 'off'); end
    
    imgMap.ButtonDownFcn = @cb_MapClick;
    cb_GenerateMap(ddMap, []);

    %% --- 3. REINFORCEMENT LEARNING CORE ---
    function cb_TrainAI(~, ~)
        update_log(1, "Initializing Q-Matrix..."); pause(0.1);
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
        btnTrain.Text = 'SYSTEM OPTIMIZED'; btnTrain.Enable = 'off';
        
        figT = figure('Name', 'Convergence Metrics', 'NumberTitle', 'off', 'Position', [600, 500, 400, 300], 'MenuBar', 'none', 'Color', 'k');
        h = heatmap(Q_Table, 'Colormap', jet);
        h.Title = 'Policy: Utility vs Cost'; h.XLabel = 'Dist Cost'; h.YLabel = 'Battery';
        h.FontColor = 'w';
        
        update_log(1, "Policy Converged.");
        update_log(2, "Weights Loaded.");
        update_log(3, "Ready.");
    end

    function q_val = get_q_value(battery_pct, path_len)
        b_idx = ceil(battery_pct / 20); if b_idx < 1, b_idx = 1; end
        d_idx = ceil(path_len / 5); if d_idx > 5, d_idx = 5; end
        q_val = Q_Table(b_idx, d_idx);
    end

    %% --- 4. LOGIC ENGINE ---
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
        if ~isTrained, uialert(fig, 'Initialize AI System First.', 'System Locked'); return; end
        coords = event.IntersectionPoint(1:2); c = round(coords(1)); r = round(coords(2));
        if r<1||r>GRID_SIZE||c<1||c>GRID_SIZE||map_matrix(r,c)==1, return; end
        idx = size(victims_data, 1) + 1; victims_data(idx, :) = [r, c, 0]; 
        obj_victims(idx).XData = c; obj_victims(idx).YData = r; obj_victims(idx).Visible = 'on';
        btnDeploy.Enable = 'on';
    end

    function cb_Deploy(~, ~)
        isRunning = true; btnDeploy.Enable = 'off'; mission_tic = tic;
        
        while isRunning && isvalid(fig)
            run_rl_allocation();
            all_docked = true;
            pending = sum(victims_data(:,3) == 0); active = sum(victims_data(:,3) == 1);
            
            for id=1:3
                update_agent(id);
                if agents_state(id) ~= "IDLE" && agents_state(id) ~= "DEAD", all_docked = false; end
            end
            
            lblTimer.Text = sprintf('T+ %.1fs', toc(mission_tic));
            
            dead_drones = 0; for k=1:3, if agents_state(k)=="DEAD", dead_drones=1; end; end
            
            if pending == 0 && active == 0 && all_docked && dead_drones == 0
                isRunning = false;
                uialert(fig, sprintf('Operation Success!\nTime: %.2fs', toc(mission_tic)), 'Mission Complete');
                break;
            end
            
            alive = 0; for k=1:3, if agents_bat(k)>0, alive=alive+1; end; end
            if alive == 0, isRunning=false; uialert(fig, 'All Units Disabled.', 'Critical Failure'); break; end
            
            drawnow; pause(0.02); 
        end
        if isvalid(fig), btnReset.Enable = 'on'; end
    end

    function run_rl_allocation()
        dead_drone_id = -1;
        for k=1:3, if agents_state(k)=="DEAD", dead_drone_id=k; break; end; end
        
        for id = 1:3
            if agents_state(id) == "DEAD" || agents_state(id) == "BEING_CARRIED", continue; end
            
            if dead_drone_id ~= -1 && dead_drone_id ~= id && agents_cargo(id) == 0
                is_rescued = false; for r=1:3, if agents_task_id(r) == -dead_drone_id, is_rescued=true; end; end
                if ~is_rescued && (agents_state(id)=="IDLE" || agents_state(id)=="MOVING")
                    agents_state(id) = "CARRYING_DRONE"; agents_task_id(id) = -dead_drone_id;
                    update_log(id, sprintf("MAYDAY! Rescuing Unit %d.", dead_drone_id));
                    continue; 
                end
            end
            
            if agents_bat(id) <= 0, continue; end
            
            if (agents_state(id) == "IDLE" || agents_state(id) == "MOVING") && agents_cargo(id) < MAX_CAPACITY
                if agents_state(id) == "MOVING" && agents_task_id(id) > 0 && victims_data(agents_task_id(id), 3) == 1, continue; end
                
                best_q = -Inf; best_v_idx = -1;
                pending_idxs = find(victims_data(:,3) == 0);
                
                for v_idx = pending_idxs'
                    v_pos = victims_data(v_idx, 1:2);
                    dist_h = sum(abs(agents_pos(id,:) - v_pos));
                    clustering = 0; if dist_h < 5, clustering = 50; end
                    q_val = get_q_value(agents_bat(id), dist_h) + clustering;
                    if q_val > best_q && q_val > 0, best_q = q_val; best_v_idx = v_idx; end
                end
                
                if best_v_idx ~= -1
                    victims_data(best_v_idx, 3) = 1; 
                    agents_state(id) = "MOVING"; agents_task_id(id) = best_v_idx;
                    agents_path{id} = bfs_path(agents_pos(id,:), victims_data(best_v_idx, 1:2));
                    update_log(id, sprintf("Target Victim #%d (Q:%.1f)", best_v_idx, best_q));
                elseif agents_cargo(id) > 0
                    agents_state(id) = "CARRYING_VICTIM";
                    agents_path{id} = bfs_path(agents_pos(id,:), HOSPITAL_POS);
                    update_log(id, "Sector Clear. RTB Hospital.");
                end
            end
        end
    end

    function update_agent(id)
        if agents_bat(id) <= 0
            if agents_state(id) ~= "DEAD", agents_state(id)="DEAD"; obj_agents(id).MarkerFaceColor='k'; update_log(id, "CRITICAL: POWER LOSS."); end
            return; 
        end
        
        if isequal(agents_pos(id,:), CHARGERS(id,:)), agents_bat(id) = min(100, agents_bat(id)+10);
        else, agents_bat(id) = max(0, agents_bat(id)-0.8); end
        
        if agents_bat(id) < 25 && ~contains(agents_state(id), ["RETURN", "IDLE", "CARRY", "DEAD", "BEING"])
             agents_state(id) = "RETURN_CHARGER";
             agents_path{id} = bfs_path(agents_pos(id,:), CHARGERS(id,:));
             update_log(id, sprintf("LOW POWER (%d%%). RTB.", floor(agents_bat(id))));
             if agents_task_id(id) > 0, victims_data(agents_task_id(id), 3)=0; agents_task_id(id)=0; end
        end
        
        path = agents_path{id};
        if ~isempty(path)
            obj_paths(id).XData=path(:,2); obj_paths(id).YData=path(:,1); obj_paths(id).Visible='on';
            step=path(1,:); agents_pos(id,:)=step;
            obj_agents(id).XData=step(2); obj_agents(id).YData=step(1);
            path(1,:)=[]; agents_path{id}=path;
            if agents_state(id) == "RETURN_CHARGER" && agents_task_id(id) < 0
                did = abs(agents_task_id(id)); agents_pos(did,:)=step; 
                obj_agents(did).XData=step(2); obj_agents(did).YData=step(1);
            end
        else, obj_paths(id).Visible='off'; end
        
        if isempty(path)
            if agents_state(id) == "MOVING"
                v = agents_task_id(id);
                obj_victims(v).Visible='off'; victims_data(v,3)=2; 
                agents_cargo(id)=agents_cargo(id)+1; agents_task_id(id)=0; 
                if agents_cargo(id)>=MAX_CAPACITY
                    agents_state(id)="CARRYING_VICTIM"; agents_path{id}=bfs_path(agents_pos(id,:), HOSPITAL_POS);
                    update_log(id, "Capacity Max. Returning.");
                else, agents_state(id)="IDLE"; end
            elseif agents_state(id) == "CARRYING_VICTIM"
                count=agents_cargo(id); agents_cargo(id)=0;
                curr=str2double(extractAfter(lblScore.Text, ': ')); lblScore.Text=sprintf('SAVED: %d', curr+count);
                agents_state(id)="RETURN_CHARGER"; agents_path{id}=bfs_path(agents_pos(id,:), CHARGERS(id,:));
                update_log(id, "Payload Delivered. RTB.");
            elseif agents_state(id) == "CARRYING_DRONE"
                did = abs(agents_task_id(id)); agents_state(did)="BEING_CARRIED";
                agents_state(id)="RETURN_CHARGER"; agents_path{id}=bfs_path(agents_pos(id,:), CHARGERS(id,:));
                update_log(id, "Unit Secured. RTB.");
            elseif agents_state(id) == "RETURN_CHARGER"
                if agents_task_id(id) < 0
                    did = abs(agents_task_id(id)); agents_state(did)="IDLE"; agents_bat(did)=50; 
                    agents_task_id(id)=0; update_log(id, "Unit Revived.");
                end
                agents_state(id)="IDLE"; update_log(id, "Docked.");
            end
        end
        
        if agents_cargo(id)>0, obj_agents(id).MarkerFaceColor='w'; 
        elseif agents_task_id(id)<0, obj_agents(id).MarkerFaceColor='y';
        else, obj_agents(id).MarkerFaceColor=agent_colors{id}; end
    end

    function update_log(id, msg)
        if isstring(msg), msg = char(msg); end
        bat = floor(agents_bat(id)/10);
        bar = repmat('|', 1, bat);
        msg_str = sprintf('[%s] %s', bar, msg);
        
        if id==1, t_log = logA; elseif id==2, t_log = logB; elseif id==3, t_log = logC; else, return; end
        
        current = t_log.Items;
        if isstring(current), current = cellstr(current); end
        if ischar(current), current = {current}; end
        if isempty(current), current = {}; end
        
        t_log.Items = [{msg_str}, reshape(current, 1, [])];
    end

    function path = bfs_path(start, goal)
        if isequal(start, goal), path=[]; return; end
        q=[start]; came_from=zeros(GRID_SIZE, GRID_SIZE, 2);
        visited=false(GRID_SIZE); visited(start(1), start(2))=true;
        found=false; dirs=[-1 0; 1 0; 0 -1; 0 1];
        while ~isempty(q)
            curr=q(1,:); q(1,:)=[];
            if isequal(curr, goal), found=true; break; end
            for k=1:4
                next=curr+dirs(k,:);
                if next(1)>0 && next(1)<=GRID_SIZE && next(2)>0 && next(2)<=GRID_SIZE
                    if map_matrix(next(1), next(2))==0 && ~visited(next(1), next(2))
                        visited(next(1), next(2))=true; came_from(next(1), next(2), :)=curr; q=[q; next];
                    end
                end
            end
        end
        if found
            path=goal; curr=goal;
            while ~isequal(curr, start)
                p=squeeze(came_from(curr(1), curr(2), :))'; path=[p; path]; curr=p;
            end
            path(1,:)=[];
        else, path=[]; end
    end

    function cb_Reset(~, ~)
        isRunning=false; victims_data=zeros(0,3);
        agents_pos=CHARGERS; agents_bat=[100, 100, 100];
        agents_state=["IDLE", "IDLE", "IDLE"]; agents_task_id=[0, 0, 0]; agents_cargo=[0, 0, 0];
        lblScore.Text='SAVED: 0'; lblTimer.Text='T+ 00.0s';
        
        init_log = {'[SYSTEM] Ready.'};
        logA.Items=init_log; logB.Items=init_log; logC.Items=init_log;
        
        for i=1:20, obj_victims(i).Visible='off'; end
        for k=1:3
            obj_agents(k).XData=CHARGERS(k,2); obj_agents(k).YData=CHARGERS(k,1);
            obj_agents(k).MarkerFaceColor=agent_colors{k}; agents_path{k}=[]; obj_paths(k).Visible='off';
        end
        btnDeploy.Enable='off'; ddMap.Enable='on';
    end
end