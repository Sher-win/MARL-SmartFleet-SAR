function multi_robot_sar_main
clc; clear; close all;

%% ----------------- 1. Create Grid Map ------------------------
mapSize = 20;
baseMap = zeros(mapSize);

% Obstacles
baseMap(8:12, 7:14)  = 1;
baseMap(4:6, 4:8)    = 1;
baseMap(14:16, 3:7)  = 1;

%% ----------------- 2. Robots ---------------------------------
robots = [
    2  2;
    2 18;
    18 10
];
numRobots = size(robots,1);

for r = 1:numRobots
    if baseMap(robots(r,1), robots(r,2)) == 1
        error('Robot %d is inside an obstacle!', r);
    end
end

victims = zeros(0,2);
colors = {'b','g','m'};
maxVictims = 10;
robotPaths = cell(numRobots,1);

%% ----------------- 3.  GUI --------------------------
fig = figure( ...
    'Name','Multi-Robot SAR (Interactive)', ...
    'Color', [0.92 0.92 0.92], ...
    'NumberTitle','off', ...
    'MenuBar','none', ...
    'ToolBar','none');

tiled = tiledlayout(fig,1,2,"Padding","compact","TileSpacing","compact");

axStatic = nexttile(tiled,1);
axAnim   = nexttile(tiled,2);

drawStatic();
drawAnimInitial();

title(axStatic,'Path Planning Panel','FontSize',13,'FontWeight','bold');
title(axAnim,'Animation Panel','FontSize',13,'FontWeight','bold');

% ---------------- Buttons ----------------
btnRun = uicontrol('Style','pushbutton','String','🚀 Run Planner',...
    'Units','normalized','Position',[0.82 0.82 0.15 0.07], ...
    'FontSize',11,'FontWeight','bold', ...
    'BackgroundColor',[0.2 0.8 0.4], ...
    'ForegroundColor','white', ...
    'Callback',@onRunPressed);

btnClear = uicontrol('Style','pushbutton','String','🧹 Clear Victims',...
    'Units','normalized','Position',[0.82 0.70 0.15 0.07], ...
    'FontSize',11,'BackgroundColor',[0.9 0.3 0.3], ...
    'ForegroundColor','white', ...
    'Callback',@onClearVictims);

pauseBtn = uicontrol('Style','togglebutton','String','⏸ Pause',...
    'Units','normalized','Position',[0.82 0.45 0.15 0.06], ...
    'FontSize',10,'BackgroundColor',[0.8 0.8 0.8]);

speedSlider = uicontrol('Style','slider','Min',0.25,'Max',2,'Value',1,...
    'Units','normalized','Position',[0.82 0.32 0.15 0.04]);

uicontrol('Style','text','String','Speed',...
    'Units','normalized','Position',[0.82 0.36 0.15 0.04], ...
    'BackgroundColor',[0.92 0.92 0.92],'FontSize',9);

infoText = uicontrol('Style','text', ...
    'String','Left-click inside LEFT panel to add victims.',...
    'Units','normalized','Position',[0.05 0.01 0.70 0.05], ...
    'BackgroundColor',[0.92 0.92 0.92], ...
    'FontSize',10,'HorizontalAlignment','left');

% Enable clicks on static panel
set(axStatic,'ButtonDownFcn',@onAxesClick);
set(get(axStatic,'Children'),'HitTest','off');

%% =============================================================
% Nested functions
%% =============================================================

    function drawBase(ax)
        cla(ax);
        hImg = imagesc(ax,1-baseMap);
        set(hImg,'HitTest','off');
        colormap(ax,gray);
        set(ax,'YDir','normal');
        axis(ax,'equal','tight');
        hold(ax,'on');
    end

    function drawStatic()
        drawBase(axStatic);

        % Robots
        hRob = plot(axStatic, robots(:,2), robots(:,1), ...
            'go','MarkerSize',10,'LineWidth',2);

        % Victims
        if ~isempty(victims)
            hVic = plot(axStatic, victims(:,2), victims(:,1), ...
                'rx','MarkerSize',12,'LineWidth',2);
        else
            hVic = [];
        end

        % Paths
        hPath = [];
        for r = 1:numRobots
            if isempty(robotPaths{r}), continue; end
            p = robotPaths{r};
            hPath = plot(axStatic,p(:,2),p(:,1), ...
                'Color',colors{r},'LineWidth',2);
        end

        % Restore clickability
        set(axStatic,'ButtonDownFcn',@onAxesClick);
        set(get(axStatic,'Children'),'HitTest','off');

        % Legend
        if ~isempty(hPath) && ~isempty(hVic)
            legend(axStatic,[hRob(1) hVic(1) hPath], ...
                {'Robots','Victims','Paths'}, ...
                'Location','southoutside','Orientation','horizontal');
        elseif ~isempty(hVic)
            legend(axStatic,[hRob(1) hVic(1)], ...
                {'Robots','Victims'});
        else
            legend(axStatic,hRob(1),'Robots');
        end
    end

    function drawAnimInitial()
        drawBase(axAnim);
        plot(axAnim, robots(:,2), robots(:,1), ...
             'go','MarkerSize',10,'LineWidth',2);
        if ~isempty(victims)
            plot(axAnim, victims(:,2), victims(:,1), ...
                 'rx','MarkerSize',12,'LineWidth',2);
        end
    end

%% ---------------- CLICK to add victim --------------------------
    function onAxesClick(~,~)
        cp = get(axStatic,'CurrentPoint');
        x = round(cp(1,1));
        y = round(cp(1,2));

        if x<1 || x>mapSize || y<1 || y>mapSize
            return;
        end
        row=y; col=x;

        if baseMap(row,col)==1
            set(infoText,'String','❌ Cannot place victim inside obstacle.');
            return;
        end
        if ismember([row col],robots,'rows')
            set(infoText,'String','❌ Cannot place victim on robot.');
            return;
        end
        if ismember([row col],victims,'rows')
            set(infoText,'String','⚠ Already exists.');
            return;
        end
        if size(victims,1) >= maxVictims
            set(infoText,'String','⚠ Max victims reached.');
            return;
        end

        victims(end+1,:) = [row col];
        robotPaths = cell(numRobots,1);
        drawStatic();
        drawAnimInitial();

        set(infoText,'String', ...
            sprintf('Added victim %d at (%d,%d).',size(victims,1),row,col));
    end

%% ---------------- CLEAR victims -------------------------------
    function onClearVictims(~,~)
        victims(:,:) = [];
        robotPaths = cell(numRobots,1);
        drawStatic();
        drawAnimInitial();
        set(infoText,'String','Victims cleared.');
    end

%% ---------------- RUN Planner --------------------------------
    function onRunPressed(~,~)
        if isempty(victims)
            set(infoText,'String','Add victims first.');
            return;
        end

        set(infoText,'String','Planning...');
        drawnow;

        [robotTasks, ~] = closestAssignment(robots, victims, baseMap);
        robotPaths = cooperativePlanning(baseMap, robots, victims, robotTasks);

        drawStatic();
        animateAndRecord(baseMap, robots, victims, robotPaths);

        set(infoText,'String','Done! You can add new victims.');
    end

%% ---------------- Closest robot assignment --------------------
    function [robotTasks,costMatrix] = closestAssignment(robotsLocal,victimsLocal,mapLocal)
        numR=size(robotsLocal,1); 
        numV=size(victimsLocal,1);
        costMatrix = inf(numR,numV);

        for r=1:numR
            for v=1:numV
                p=astar(mapLocal,robotsLocal(r,:),victimsLocal(v,:));
                if ~isempty(p), costMatrix(r,v)=size(p,1)-1; end
            end
        end

        assignment=zeros(numV,1);
        for v=1:numV
            [val,idx]=min(costMatrix(:,v));
            if ~isinf(val), assignment(v)=idx; end
        end

        robotTasks = cell(numR,1);
        for r=1:numR
            myVicts=find(assignment==r);
            if isempty(myVicts), continue; end

            seq=[];
            curPos=robotsLocal(r,:);
            remaining=myVicts(:)';
            while ~isempty(remaining)
                bestV=remaining(1);
                bestD=inf;
                for v=remaining
                    d=abs(curPos(1)-victimsLocal(v,1))+ ...
                      abs(curPos(2)-victimsLocal(v,2));
                    if d<bestD, bestD=d; bestV=v; end
                end
                seq=[seq bestV]; %#ok<AGROW>
                curPos=victimsLocal(bestV,:);
                remaining(remaining==bestV)=[];
            end
            robotTasks{r}=seq;
        end
    end

%% ---------------- Cooperative A* ------------------------------
    function robotPathsOut = cooperativePlanning(mapLocal,robotsLocal,victimsLocal,robotTasks)
        robotPathsOut = cell(numRobots,1);
        reservedMap = mapLocal;

        for r=1:numRobots
            seq=robotTasks{r};
            curPos=robotsLocal(r,:);
            fullPath=curPos;
            localMap=reservedMap;

            for k=1:numel(seq)
                vIdx=seq(k);
                goal=victimsLocal(vIdx,:);
                p=astar(localMap,curPos,goal);
                if isempty(p), p=astar(mapLocal,curPos,goal); end
                if isempty(p), break; end
                fullPath=[fullPath; p(2:end,:)]; %#ok<AGROW>
                curPos=goal;
            end

            back=astar(localMap,curPos,robotsLocal(r,:));
            if isempty(back), back=astar(mapLocal,curPos,robotsLocal(r,:)); end
            if ~isempty(back)
                fullPath=[fullPath; back(2:end,:)]; %#ok<AGROW>
            end

            robotPathsOut{r}=fullPath;

            for i=1:size(fullPath,1)
                reservedMap(fullPath(i,1),fullPath(i,2))=1;
            end
        end
    end

%% ---------------- Animation + video ---------------------------
    function animateAndRecord(mapLocal,robotsLocal,victimsLocal,robotPathsLocal)
        makeVideo=true;
        if makeVideo
            vOut=VideoWriter('multi_robot_sar.mp4','MPEG-4');
            vOut.FrameRate=10; open(vOut);
        end

        currentPos=nan(numRobots,2);
        currentIdx=zeros(numRobots,1);
        maxLen=0;

        for r=1:numRobots
            p=robotPathsLocal{r};
            if isempty(p),continue; end
            currentPos(r,:)=p(1,:);
            currentIdx(r)=1;
            maxLen=max(maxLen,size(p,1));
        end

        for t=1:maxLen
            if get(pauseBtn,'Value')==1
                set(infoText,'String','Paused');
                while get(pauseBtn,'Value')==1, pause(0.1); end
            end

            % update positions
            for r=1:numRobots
                p=robotPathsLocal{r};
                if isempty(p),continue; end
                if currentIdx(r)<size(p,1)
                    currentIdx(r)=currentIdx(r)+1;
                    currentPos(r,:)=p(currentIdx(r),:);
                end
            end

            % draw animation panel
            drawBase(axAnim);
            if ~isempty(victimsLocal)
                plot(axAnim,victimsLocal(:,2),victimsLocal(:,1),'rx','MarkerSize',12,'LineWidth',2);
            end

            for r=1:numRobots
                p=robotPathsLocal{r};
                if isempty(p),continue; end
                trail=p(1:currentIdx(r),:);
                plot(axAnim,trail(:,2),trail(:,1),'Color',colors{r},'LineWidth',2);
                pos=currentPos(r,:);
                plot(axAnim,pos(2),pos(1),'o','Color',colors{r}, ...
                    'MarkerFaceColor',colors{r},'MarkerSize',10,'LineWidth',2);
            end

            % draw bases
            plot(axAnim,robotsLocal(:,2),robotsLocal(:,1),'go','MarkerSize',10,'LineWidth',2);
            title(axAnim,sprintf('Time step %d',t),'FontSize',12);

            drawnow;

            if makeVideo
                frame=getframe(fig);
                writeVideo(vOut,frame);
            end

            pause(0.15/get(speedSlider,'Value'));
        end

        if makeVideo
            close(vOut);
            set(infoText,'String','🎥 Video saved!');
        end
    end
end
