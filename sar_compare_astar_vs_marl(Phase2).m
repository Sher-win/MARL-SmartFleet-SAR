function sar_compare_astar_vs_marl
%% ============================================================
%  A* vs MARL Comparison — FINAL ROBUST VERSION
% ============================================================
clc; close all;

%% --------- Load trained MARL agents safely --------------------------
if ~isfile("marl_dqn_agents.mat")
    error("File 'marl_dqn_agents.mat' not found. Please train agents first.");
end
S = load("marl_dqn_agents.mat");
agents = S.agents;

%% --------- Base environment (for map + robot starts) ----------------
env0       = EnvSAR_MARL();
map        = env0.map;
robotStart = env0.robotStart;
numRobots  = env0.numRobots;

%% --------- Robot colors ---------------------------------------------
robotColors = [
    0    1    1;   % Cyan
    1    0    1;   % Magenta
    1    1    0    % Yellow
];

%% --------- Shared data stored in base workspace ----------------------
assignin('base','victims',[]);
assignin('base','astarTrailX',cell(numRobots,1));
assignin('base','astarTrailY',cell(numRobots,1));
assignin('base','marlTrailX', cell(numRobots,1));
assignin('base','marlTrailY', cell(numRobots,1));

%% ================= FIGURE & LAYOUT ==================================
fig = figure('Name','A* vs MARL Comparison',...
    'NumberTitle','off','Color','black','MenuBar','none');
t = tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');

%% -------- Left: A* panel --------
axA = nexttile(t,1);
title(axA,'A* Cooperative','Color','white','FontSize',16,'FontWeight','bold');
axis(axA,[1 20 1 20]); axis square; hold(axA,'on'); set(axA,'Color','white');
drawMap(axA,map);

%% -------- Right: MARL panel --------
axR = nexttile(t,2);
title(axR,'MARL (DQN)','Color','white','FontSize',16,'FontWeight','bold');
axis(axR,[1 20 1 20]); axis square; hold(axR,'on'); set(axR,'Color','white');
drawMap(axR,map);

%% Initial draw (no victims)
redrawAll(axA,axR,map,robotStart,robotColors,[]);

%% ================= CLICK TO ADD VICTIMS ===============================
set(fig,'WindowButtonDownFcn',...
    @(src,evnt) clickAddVictim(axA,axR,map,robotStart,robotColors));

%% ================= START BUTTON ======================================
uicontrol('Style','pushbutton','String','START',...
    'Units','normalized','Position',[0.40 0.05 0.18 0.07],...
    'FontSize',14,'BackgroundColor',[0 0.6 0],'ForegroundColor','white',...
    'Callback',@(src,evnt) runComparison(axA,axR,map,robotStart,robotColors,agents));

%% ================= CLEAR BUTTON ======================================
uicontrol('Style','pushbutton','String','CLEAR',...
    'Units','normalized','Position',[0.65 0.05 0.18 0.07],...
    'FontSize',14,'BackgroundColor',[0.6 0 0],'ForegroundColor','white',...
    'Callback',@(src,evnt) clearAll(axA,axR,map,robotStart,robotColors));

end % END MAIN FUNCTION

%% ====================================================================
% CLICK HANDLER → add victim
% =====================================================================
function clickAddVictim(axA,axR,map,robotStart,robotColors)
    victims = evalin('base','victims');
    cp = get(axA,'CurrentPoint');
    x = round(cp(1,1)); 
    y = round(cp(1,2));
    if x<1 || x>20 || y<1 || y>20, return; end
    if map(y,x)==1
        disp("Cannot place victim on obstacle.");
        return;
    end
    victims(end+1,:) = [y x];
    assignin('base','victims',victims);
    disp("Victim added at ("+y+","+x+")");
    redrawAll(axA,axR,map,robotStart,robotColors,victims);
end

%% ====================================================================
% CLEAR EVERYTHING (victims + all trails)
% =====================================================================
function clearAll(axA,axR,map,robotStart,robotColors)
    assignin('base','victims',[]);
    assignin('base','astarTrailX',cell(size(robotStart,1),1));
    assignin('base','astarTrailY',cell(size(robotStart,1),1));
    assignin('base','marlTrailX',cell(size(robotStart,1),1));
    assignin('base','marlTrailY',cell(size(robotStart,1),1));
    redrawAll(axA,axR,map,robotStart,robotColors,[]);
    disp("Cleared victims and trails.");
end

%% ====================================================================
% RUN BOTH (A* then MARL)
% =====================================================================
function runComparison(axA,axR,map,robotStart,robotColors,agents)
    victims = evalin('base','victims');
    if isempty(victims)
        disp("Add victims first!");
        return;
    end
    
    disp("[A*] Running cooperative victim assignment...");
    runAstar(axA,map,robotStart,robotColors,victims);
    
    pause(0.4);
    
    disp("[MARL] Starting simulation...");
    runMarl(axR,map,robotStart,robotColors,agents,victims);
end

%% ====================================================================
% MARL EXECUTION — (FIXED: Handles INF Inputs)
% =====================================================================
function runMarl(ax,map,robotStart,robotColors,agents,victims)
    marlX = evalin('base','marlTrailX');
    marlY = evalin('base','marlTrailY');

    env = EnvSAR_MARL();
    env.victims = victims;
    env.numVictims = size(victims,1);
    [states, env] = env.reset();
    numRobots = env.numRobots;
    MAX_T = 100;
    for t = 1:MAX_T
        actions = zeros(numRobots,1);
         % To check if any more victims remain and if the job is done
        if isempty(env.victimStatus)
            remainingCount = 0;
        else
            remainingCount = sum(env.victimStatus == 0);
        end
        
        if remainingCount == 0 % STOP EVERYONE
            actions(:) = 5; 
            done = true;
        else % INDIVIDUAL AGENT DECISIONS
            for r = 1:numRobots
                distToVictim = states(r,5); % Index 5 is Distance
                
                % ⭐ CRITICAL FIX: If distance is Infinite (no path/no victim),
                % the Neural Network will crash/output garbage. Force STAY.
                if distToVictim == Inf || isnan(distToVictim)
                    actions(r) = 5; % Stay
                else
                    q = dqn_helpers("forward",agents{r},states(r,:));
                    [~,act] = max(q);
                    actions(r) = act;
                end
            end
            done = false;
        end

        % Step Environment (only if not forcefully done)
        if ~done
            [states,~,~,done,env] = env.step(actions);
        end
        
        % Log Trails
        for r = 1:numRobots
            pos = env.robotPos(r,:);
            marlX{r}(end+1) = pos(2);
            marlY{r}(end+1) = pos(1);
        end
        
        assignin('base','marlTrailX',marlX);
        assignin('base','marlTrailY',marlY);
        redrawAll(ax,ax,map,robotStart,robotColors,victims);
        drawnow; pause(0.04);
        
        if done
            disp("[MARL] Finished (All rescued).");
            break;
        end
    end
end

%% ====================================================================
% A* EXECUTION
% =====================================================================
function runAstar(ax,map,robotStart,robotColors,victims)
    astarX = evalin('base','astarTrailX');
    astarY = evalin('base','astarTrailY');
    assigned = assignAstar(map,robotStart,victims);
    numRobots = size(robotStart,1);
    
    for r = 1:numRobots
        seq = assigned{r};
        pos = robotStart(r,:);
        fullPath = pos;
        for v = seq
            p = astar(map,pos,victims(v,:));
            if isempty(p), continue; end
            fullPath = [fullPath; p(2:end,:)];
            pos = victims(v,:);
        end
        % Optional: Go back to start? (Commented out for comparison parity)
        % back = astar(map,pos,robotStart(r,:));
        % if ~isempty(back), fullPath = [fullPath; back(2:end,:)]; end
        
        for i = 1:size(fullPath,1)
            p = fullPath(i,:);
            astarX{r}(end+1) = p(2);
            astarY{r}(end+1) = p(1);
            assignin('base','astarTrailX',astarX);
            assignin('base','astarTrailY',astarY);
            redrawAll(ax,ax,map,robotStart,robotColors,victims);
            drawnow; pause(0.04);
        end
    end
end

%% ====================================================================
% Helper: Redraw
% =====================================================================
function redrawAll(axA,axR,map,robotStart,robotColors,victims)
    astarX = evalin('base','astarTrailX');
    astarY = evalin('base','astarTrailY');
    marlX  = evalin('base','marlTrailX');
    marlY  = evalin('base','marlTrailY');
    
    cla(axA); drawMap(axA,map);
    cla(axR); drawMap(axR,map);
    
    if ~isempty(victims)
        plot(axA,victims(:,2),victims(:,1),'rx','MarkerSize',12,'LineWidth',2);
        plot(axR,victims(:,2),victims(:,1),'rx','MarkerSize',12,'LineWidth',2);
    end
    
    numRobots = size(robotStart,1);
    for r=1:numRobots
        if ~isempty(astarX{r})
            plot(axA,astarX{r},astarY{r},'-','Color',robotColors(r,:)*0.6,'LineWidth',2);
        end
        if ~isempty(marlX{r})
            plot(axR,marlX{r},marlY{r},'-','Color',robotColors(r,:)*0.6,'LineWidth',2);
        end
        plot(axA,robotStart(r,2),robotStart(r,1),'o','MarkerSize',10,...
            'MarkerFaceColor',robotColors(r,:),'MarkerEdgeColor','black','LineWidth',2);
        plot(axR,robotStart(r,2),robotStart(r,1),'o','MarkerSize',10,...
            'MarkerFaceColor',robotColors(r,:),'MarkerEdgeColor','black','LineWidth',2);
    end
end

function drawMap(ax,map)
    imagesc(ax,1-map);
    colormap(ax,gray);
    set(ax,'YDir','normal');
    axis(ax,[1 20 1 20]); axis square;
    grid(ax,'on'); hold(ax,'on');
    ax.GridColor = [.5 .5 .5];
    ax.GridAlpha = .2;
end

function assigned = assignAstar(map,startPos,victims)
    numR = size(startPos,1);
    numV = size(victims,1);
    cost = inf(numR,numV);
    for r=1:numR
        for v=1:numV
            p = astar(map,startPos(r,:),victims(v,:));
            if ~isempty(p)
                cost(r,v) = size(p,1);
            end
        end
    end
    assigned = cell(numR,1);
    for v=1:numV
        [~,bestR] = min(cost(:,v));
        assigned{bestR}(end+1) = v;
    end
end