%% POTENTIAL FIELD-BASED MOBILE ROBOT
% 3 Static + 2 Moving Obstacles - random scenario (fixed seed by default)
% The moving obstacles patrol back and forth ACROSS the robot's route.
% They never react to the robot: the robot does all the avoiding.

% MODEL ASSUMPTIONS
%  - Point-mass kinematic robot: constant step length, instant heading change.
%  - Robot knows obstacle positions and velocities exactly (perfect sensing).
%  - Moving obstacles follow fixed patrol lines and never react to the robot.
%  - Tangential force and the escape routine are heuristics added to plain APF
%    to handle local minima; they are not part of the plotted potential.
%  - A hard collision check (section 5g) acts as a safety layer on top of APF.
%  - Repulsion fades out near the goal (GNRON fix, section 5c).
%  - Units are arbitrary length units (not metres).
%  - Section 9 maps are a snapshot at the final time (moving obstacles at
%    their last positions), not the field the robot saw during the run.

clc;
clear;
close all;

%% 1. PARAMETERS

randomScenario = false;   % false = repeatable scenario (seed 10), true = new one every run
if randomScenario
    rng('shuffle');
else
    rng(10);
end

startPosition = [1, 1];
goal = [9, 9];

numStatic = 3;
numMoving = 2;
numObstacles = numStatic + numMoving;

staticRadius = 0.55;
movingRadius = 0.45;
robotRadius = 0.16;

Katt = 1.2;
Krep = 1.0;
d0 = 1.6;

dt = 0.05;                 % time step [s]
stepSize = 0.04;           % robot speed = stepSize/dt = 0.8 units/s
maxSteps = 6000;
goalTolerance = 0.40;
drawEvery = 3;             % redraw graphics every N steps (speed)
saveOutputs = true;        % false = skip saving PNG/CSV files (faster finish)

goalFadeRadius = 1.0;      % repulsion fades to 0 as robot reaches goal (GNRON fix)

tPredict = 0.8;            % look-ahead for moving obstacles [s]
wPredict = 0.6;            % weight of predicted-position repulsion

stuckLimit = 70;
escapeDuration = 55;
escapeSteps = 0;
escapeSide = 1;
stuckCounter = 0;
recoveryAttempts = 0;
maxRecoveryAttempts = 8;

%% 2. GENERATE OBSTACLES
% Route direction (start -> goal)
routeDir = (goal-startPosition)/norm(goal-startPosition);
perpDir  = [-routeDir(2), routeDir(1)];
routeAt  = @(t) startPosition + t*(goal-startPosition);

obsPos = zeros(numObstacles,2);
obsVel = zeros(numObstacles,2);
obsR   = [staticRadius*ones(numStatic,1); movingRadius*ones(numMoving,1)];
isMoving = [false(numStatic,1); true(numMoving,1)];

% Moving-obstacle patrol data
patrolC  = zeros(numMoving,2);   % patrol centre (on the route)
patrolL  = 2.2*ones(numMoving,1);% half-length of patrol line
patrolSp = zeros(numMoving,1);   % speed
patrolS  = zeros(numMoving,1);   % current offset along patrol line
patrolSg = ones(numMoving,1);    % direction sign

valid = false;
tries = 0;
while ~valid && tries < 2000
    tries = tries + 1;

    % Moving obstacles: patrol perpendicular to the route
    tMov = [0.38 + 0.06*(rand-0.5), 0.72 + 0.06*(rand-0.5)];
    for m = 1:numMoving
        patrolC(m,:)  = routeAt(tMov(m));
        patrolSp(m)   = 0.45 + 0.25*rand();
        patrolS(m)    = patrolL(m)*(2*rand-1);
        patrolSg(m)   = sign(rand-0.5) + (rand==0.5);
    end

    % Static obstacles: near the route with random lateral offset
    tSt = [0.18, 0.52, 0.87] + 0.04*(rand(1,3)-0.5);
    for k = 1:numStatic
        obsPos(k,:) = routeAt(tSt(k)) + perpDir*(-0.7 + 1.4*rand());
    end

    % Validate: statics clear of start/goal, of each other, and of patrol lines
    valid = true;
    for k = 1:numStatic
        if norm(obsPos(k,:)-startPosition) < 1.5 || norm(obsPos(k,:)-goal) < 1.5
            valid = false;
        end
        for j = k+1:numStatic
            if norm(obsPos(k,:)-obsPos(j,:)) < 1.8
                valid = false;
            end
        end
        for m = 1:numMoving
            a = patrolC(m,:) - perpDir*patrolL(m);
            b = patrolC(m,:) + perpDir*patrolL(m);
            ab = b-a;
            u = max(0,min(1,dot(obsPos(k,:)-a,ab)/dot(ab,ab)));
            if norm(obsPos(k,:)-(a+u*ab)) < staticRadius+movingRadius+1.2
                valid = false;
            end
        end
    end
end
if ~valid
    warning('Could not generate a valid obstacle layout in %d tries.',tries);
end

% Initial moving-obstacle state
for m = 1:numMoving
    idx = numStatic+m;
    obsPos(idx,:) = patrolC(m,:) + perpDir*patrolS(m);
    obsVel(idx,:) = patrolSg(m)*patrolSp(m)*perpDir;
end

fprintf('\nObstacles [X Y Radius VX VY] (first %d static, last %d moving):\n', ...
    numStatic,numMoving);
disp([obsPos obsR obsVel]);

%% 3. INITIALIZE ROBOT

robot = startPosition;
path = robot;
trail = cell(numMoving,1);
for m = 1:numMoving
    trail{m} = obsPos(numStatic+m,:);
end

reachedGoal = false;
bestGoalDistance = norm(goal-robot);
minClearance = inf;
collisionSteps = 0;
waitSteps = 0;
clearHist = [];
FattHist = []; FrepHist = []; FtanHist = []; FtotHist = [];
goalDistHist = [];

%% 4. CREATE 3D-STYLE ENVIRONMENT

fig3D = figure('Name','Potential Field - 2 Moving + 3 Static Obstacles', ...
       'NumberTitle','off','Color','w','Position',[40 60 900 700]);

ax3d = axes('Parent',fig3D);
hold on; grid on; axis equal;
axis([0 10 0 10 0 1.5]);
view(45,30);
xlabel('X Position'); ylabel('Y Position'); zlabel('Height');
title('Robot Navigation: 3 Static + 2 Moving Obstacles');

[Xg,Yg] = meshgrid(0:0.5:10,0:0.5:10);
surf(Xg,Yg,zeros(size(Xg)), ...
    'FaceColor',[0.92 0.95 0.92],'EdgeColor',[0.82 0.84 0.82]);

theta = linspace(0,2*pi,50);
[Theta,Zc] = meshgrid(theta,[0 0.9]);

hCyl = gobjects(numObstacles,1);
hSafe = gobjects(numObstacles,1);
hVel = gobjects(numMoving,1);
staticColor = [0.55 0.55 0.6];
movingColor = [1.0 0.5 0.1];

for i = 1:numObstacles
    if isMoving(i), col = movingColor; else, col = staticColor; end

    hCyl(i) = surf(obsPos(i,1)+obsR(i)*cos(Theta), ...
                   obsPos(i,2)+obsR(i)*sin(Theta),Zc, ...
        'FaceColor',col,'EdgeColor','none','FaceAlpha',0.9);

    hSafe(i) = plot3(obsPos(i,1)+(obsR(i)+robotRadius)*cos(theta), ...
                     obsPos(i,2)+(obsR(i)+robotRadius)*sin(theta), ...
                     0.03*ones(size(theta)),'r--','LineWidth',0.8);
end

% Patrol lines of moving obstacles (dotted)
for m = 1:numMoving
    a = patrolC(m,:) - perpDir*patrolL(m);
    b = patrolC(m,:) + perpDir*patrolL(m);
    plot3([a(1) b(1)],[a(2) b(2)],[0.02 0.02],':', ...
        'Color',movingColor,'LineWidth',1.2);
    idx = numStatic+m;
    hVel(m) = quiver3(obsPos(idx,1),obsPos(idx,2),0.95, ...
        obsVel(idx,1),obsVel(idx,2),0,'Color',[0.8 0.3 0], ...
        'LineWidth',1.8,'MaxHeadSize',2);
end

hStart = plot3(startPosition(1),startPosition(2),0.15,'go', ...
    'MarkerFaceColor','g','MarkerSize',10);
hGoal = plot3(goal(1),goal(2),0.15,'p','MarkerFaceColor','y', ...
    'MarkerEdgeColor','k','MarkerSize',15);
text(startPosition(1),startPosition(2),0.4,' START');
text(goal(1),goal(2),0.4,' GOAL');

robotSize = robotRadius;
hRobot = patch(robot(1)+robotSize*[-1 1 1 -1], ...
    robot(2)+robotSize*[-1 -1 1 1],0.25*ones(1,4),[0.1 0.35 0.95], ...
    'EdgeColor','k','LineWidth',1.2);
hHeading = plot3([robot(1),robot(1)+0.25],[robot(2),robot(2)+0.25], ...
    [0.3 0.3],'b-','LineWidth',2);
hPath = plot3(robot(1),robot(2),0.12,'b-','LineWidth',2);
% Force colours
cAtt = [0.00 0.65 0.00];   % attractive  - green
cRep = [0.85 0.00 0.00];   % repulsive   - red
cTan = [0.80 0.00 0.80];   % tangential  - magenta
cTot = [0.00 0.00 0.00];   % resultant   - black

% Separate arrows for every force acting on the robot
hFatt = quiver3(robot(1),robot(2),0.40,0,0,0,'Color',cAtt,'LineWidth',2.2, ...
    'MaxHeadSize',1.5,'AutoScale','off');
hFrep = quiver3(robot(1),robot(2),0.45,0,0,0,'Color',cRep,'LineWidth',2.2, ...
    'MaxHeadSize',1.5,'AutoScale','off');
hFtan = quiver3(robot(1),robot(2),0.50,0,0,0,'Color',cTan,'LineWidth',2.2, ...
    'MaxHeadSize',1.5,'AutoScale','off');
hFtot = quiver3(robot(1),robot(2),0.55,0,0,0,'Color',cTot,'LineWidth',3, ...
    'MaxHeadSize',1.5,'AutoScale','off');

% Lines from robot to every obstacle inside its influence distance d0
hLink = gobjects(numObstacles,1);
for i = 1:numObstacles
    hLink(i) = plot3(nan,nan,nan,'--','Color',[0.85 0.4 0.4],'LineWidth',1);
end

legend(ax3d,[hStart,hGoal,hRobot,hPath,hFatt,hFrep,hFtan,hFtot, ...
    hCyl(1),hCyl(numStatic+1),hLink(1)], ...
    {'Start','Goal','Robot','Robot path','Attractive force', ...
     'Repulsive force','Tangential force','Resultant force', ...
     'Static obstacle','Moving obstacle','Obstacle within d_0'}, ...
    'Location','southoutside','NumColumns',4);

% Separate window for the live force and clearance plots
figLive = figure('Name','Live Forces and Clearance','NumberTitle','off', ...
    'Color','w','Position',[960 60 600 700]);

% Live force-magnitude plot
axF = subplot(2,1,1);
hold on; grid on;
hFa = plot(nan,nan,'Color',cAtt,'LineWidth',1.5);
hFr = plot(nan,nan,'Color',cRep,'LineWidth',1.5);
hFt = plot(nan,nan,'Color',cTan,'LineWidth',1.5);
hFT = plot(nan,nan,'Color',cTot,'LineWidth',2);
xlabel('Time [s]'); ylabel('Force magnitude');
title('Force components (live)');
legend(axF,{'Attractive','Repulsive','Tangential','Resultant'}, ...
    'Location','northeast');

% Live clearance plot
axC = subplot(2,1,2);
hold on; grid on;
hCl = plot(nan,nan,'b','LineWidth',1.5);
yline(0,'r--');
xlabel('Time [s]'); ylabel('Clearance [units]');
title('Distance to nearest obstacle surface (live)');

drawnow;

%% 5. ROBOT NAVIGATION LOOP

for step = 1:maxSteps

    % Save previous obstacle positions to check relative motion this time step.
    obsPosPrev = obsPos;

    %% 5a. Move ONLY the moving obstacles along their fixed patrol lines
    for m = 1:numMoving
        idx = numStatic+m;
        patrolS(m) = patrolS(m) + patrolSg(m)*patrolSp(m)*dt;
        if abs(patrolS(m)) > patrolL(m)
            patrolSg(m) = -patrolSg(m);
            patrolS(m) = sign(patrolS(m))*patrolL(m);
        end
        obsPos(idx,:) = patrolC(m,:) + perpDir*patrolS(m);
        obsVel(idx,:) = patrolSg(m)*patrolSp(m)*perpDir;
    end

    %% 5b. Attractive force
    Fatt = Katt*(goal-robot);

    %% 5c. Repulsive and tangential forces
    Frep = [0 0];
    Ftan = [0 0];
    rhoAll = inf(numObstacles,1);
    minSurfaceDistance = inf;
    nearestTangent = [0 0];

    for i = 1:numObstacles

        radius = obsR(i);

        if isMoving(i)
            centers = [obsPos(i,:); obsPos(i,:)+obsVel(i,:)*tPredict];
            weights = [1, wPredict];
        else
            centers = obsPos(i,:);
            weights = 1;
        end

        for c = 1:size(centers,1)

            center = centers(c,:);
            delta = robot-center;
            centerDistance = norm(delta);

            if centerDistance > 1e-8
                outward = delta/centerDistance;
            else
                outward = [1 0];
            end

            rho = centerDistance-radius-robotRadius;

            if c == 1
                minSurfaceDistance = min(minSurfaceDistance,rho);
                rhoAll(i) = rho;
            end

            if rho < d0

                safeRho = max(rho,0.15);
                repMagnitude = Krep*(1/safeRho-1/d0)/(safeRho^2);
                repMagnitude = min(max(repMagnitude,0),3.5);

                Frep = Frep+weights(c)*repMagnitude*outward;

                tangent = [-outward(2),outward(1)];
                if dot(tangent,goal-robot) < 0
                    tangent = -tangent;
                end
                if escapeSteps > 0
                    tangent = escapeSide*[-outward(2),outward(1)];
                end

                tangentStrength = 0.6*max(0,(d0-rho)/d0);
                Ftan = Ftan+weights(c)*tangentStrength*tangent;

                if c == 1 && norm(nearestTangent) < 1e-8
                    nearestTangent = tangent;
                end
            end
        end
    end

    % Goal-not-reachable-with-obstacles-nearby fix: fade obstacle forces
    % out as the robot approaches the goal so the goal stays a minimum.
    goalFade = min(1,norm(goal-robot)/goalFadeRadius);
    Frep = goalFade*Frep;
    Ftan = goalFade*Ftan;

    %% 5d. Resultant force
    Ftotal = Fatt+Frep+Ftan;

    if norm(Ftotal) < 0.15 && minSurfaceDistance < d0
        if norm(nearestTangent) > 1e-8
            Ftotal = nearestTangent;
        end
    end

    %% 5e. Detect lack of progress
    currentGoalDistance = norm(goal-robot);

    if currentGoalDistance < bestGoalDistance-0.01
        bestGoalDistance = currentGoalDistance;
        stuckCounter = 0;
    else
        stuckCounter = stuckCounter+1;
    end

    if stuckCounter >= stuckLimit
        escapeSide = -escapeSide;
        escapeSteps = escapeDuration;
        stuckCounter = 0;
        bestGoalDistance = currentGoalDistance;
        fprintf('Step %d: Changing escape direction.\n',step);
    end

    if escapeSteps > 0
        escapeSteps = escapeSteps-1;
    end

    %% 5f. Calculate movement
    forceMagnitude = norm(Ftotal);
    direction = [0 0];
    if forceMagnitude > 1e-8
        direction = Ftotal/forceMagnitude;
    end

    candidate = robot+stepSize*direction;

    %% 5g. Collision checking along the robot move and moving-obstacle motion
    % SAFETY LAYER (not part of the potential field): if the APF step would
    % hit an obstacle, try a sidestep along the tangent, otherwise wait.
    collision = ~segmentSafeMoving(robot,candidate, ...
        obsPosPrev,obsPos,obsR,robotRadius);

    if collision
        movedSafely = false;

        if norm(nearestTangent) > 1e-8
            directions = [nearestTangent; -nearestTangent];
            for k = 1:2
                testCandidate = robot+stepSize*directions(k,:);
                safe = segmentSafeMoving(robot,testCandidate, ...
                    obsPosPrev,obsPos,obsR,robotRadius);
                if safe
                    candidate = testCandidate;
                    movedSafely = true;
                    break;
                end
            end
        end

        if ~movedSafely
            candidate = robot;   % wait for the moving obstacle to pass
        end
    end

    %% 5h. Update position and logs
    waitSteps = waitSteps + (norm(candidate-robot) < 1e-12);
    robot = candidate;
    path(end+1,:) = robot;

    clearance = min(vecnorm(obsPos-robot,2,2)-obsR-robotRadius);
    clearHist(end+1) = clearance;
    FattHist(end+1) = norm(Fatt);
    FrepHist(end+1) = norm(Frep);
    FtanHist(end+1) = norm(Ftan);
    FtotHist(end+1) = norm(Ftotal);
    goalDistHist(end+1) = norm(goal-robot);
    minClearance = min(minClearance,clearance);
    if clearance < 0
        collisionSteps = collisionSteps+1;
    end

    for m = 1:numMoving
        trail{m}(end+1,:) = obsPos(numStatic+m,:);
    end

    %% 5i. Update graphics (only moving obstacles change)
    if mod(step,drawEvery) == 0
    for m = 1:numMoving
        i = numStatic+m;
        set(hCyl(i),'XData',obsPos(i,1)+obsR(i)*cos(Theta), ...
                    'YData',obsPos(i,2)+obsR(i)*sin(Theta));
        set(hSafe(i),'XData',obsPos(i,1)+(obsR(i)+robotRadius)*cos(theta), ...
                     'YData',obsPos(i,2)+(obsR(i)+robotRadius)*sin(theta));
        set(hVel(m),'XData',obsPos(i,1),'YData',obsPos(i,2), ...
                    'UData',obsVel(i,1)*1.5,'VData',obsVel(i,2)*1.5);
    end

    set(hRobot,'XData',robot(1)+robotSize*[-1 1 1 -1], ...
               'YData',robot(2)+robotSize*[-1 -1 1 1]);
    set(hHeading,'XData',[robot(1),robot(1)+0.3*direction(1)], ...
                 'YData',[robot(2),robot(2)+0.3*direction(2)]);
    set(hPath,'XData',path(:,1),'YData',path(:,2), ...
              'ZData',0.12*ones(size(path,1),1));

    % Force arrows (length proportional to force, capped for readability)
    setArrow(hFatt,robot,Fatt,  0.12,1.5,0.40);
    setArrow(hFrep,robot,Frep,  0.12,1.5,0.45);
    setArrow(hFtan,robot,Ftan,  0.12,1.5,0.50);
    setArrow(hFtot,robot,Ftotal,0.12,1.5,0.55);

    % Show which obstacles are currently repelling the robot
    for i = 1:numObstacles
        if rhoAll(i) < d0
            set(hLink(i),'XData',[robot(1) obsPos(i,1)], ...
                'YData',[robot(2) obsPos(i,2)],'ZData',[0.35 0.35]);
        else
            set(hLink(i),'XData',nan,'YData',nan,'ZData',nan);
        end
    end

    % Live force and clearance plots
    tNow = (1:numel(FattHist))*dt;
    set(hFa,'XData',tNow,'YData',FattHist);
    set(hFr,'XData',tNow,'YData',FrepHist);
    set(hFt,'XData',tNow,'YData',FtanHist);
    set(hFT,'XData',tNow,'YData',FtotHist);
    set(hCl,'XData',tNow,'YData',clearHist);

    title(ax3d,sprintf(['t = %.1f s | Goal distance %.2f | ', ...
        'Clearance %.2f | |Fatt| %.1f  |Frep| %.1f'], ...
        step*dt,norm(goal-robot),clearance,norm(Fatt),norm(Frep)));

    drawnow;
    pause(0.01);
    end

    %% 5j. Debug output
    if mod(step,50) == 0
        fprintf(['Step %d | Position (%.2f, %.2f) | Fatt %.2f | ', ...
            'Frep %.2f | Ftan %.2f | Total %.2f | Surface dist %.2f\n'], ...
            step,robot(1),robot(2),norm(Fatt),norm(Frep), ...
            norm(Ftan),norm(Ftotal),minSurfaceDistance);
    end

    %% 5k. Goal reached?
    if norm(goal-robot) < goalTolerance
        reachedGoal = true;
        break;
    end
end

%% 6. DISPLAY RESULTS

if reachedGoal
    title(ax3d,sprintf('GOAL REACHED in %.1f s - navigation finished', ...
        (size(path,1)-1)*dt));
else
    title(ax3d,'Goal NOT reached - navigation stopped');
end
drawnow;
fprintf('\nNavigation finished. Building analysis figures...\n');

fprintf('\n========== SIMULATION RESULTS ==========\n');
fprintf('Static obstacles: %d | Moving obstacles: %d\n',numStatic,numMoving);
fprintf('Steps taken: %d (%.1f s)\n',size(path,1)-1,(size(path,1)-1)*dt);
fprintf('Final position: (%.2f, %.2f)\n',robot(1),robot(2));
fprintf('Path length: %.2f units\n',sum(vecnorm(diff(path),2,2)));
fprintf('Minimum clearance: %.3f units | Collision steps: %d\n', ...
    minClearance,collisionSteps);

if reachedGoal
    fprintf('STATUS: GOAL REACHED SUCCESSFULLY!\n');
else
    fprintf('STATUS: GOAL NOT REACHED within the step limit.\n');
end

%% 7. FINAL TOP-DOWN VIEW

figPath = figure('Name','Final Robot Path','Color','w');
hold on; grid on; axis equal; axis([0 10 0 10]);
xlabel('X Position'); ylabel('Y Position');
title('Robot Path, Static Obstacles and Moving-Obstacle Trajectories');

th2 = linspace(0,2*pi,100);

for k = 1:numStatic
    fill(obsPos(k,1)+obsR(k)*cos(th2),obsPos(k,2)+obsR(k)*sin(th2), ...
        staticColor,'FaceAlpha',0.8,'EdgeColor','k');
    plot(obsPos(k,1)+(obsR(k)+robotRadius)*cos(th2), ...
         obsPos(k,2)+(obsR(k)+robotRadius)*sin(th2),'r--');
end

for m = 1:numMoving
    plot(trail{m}(:,1),trail{m}(:,2),'-','Color',movingColor,'LineWidth',1.2);
    fill(obsPos(numStatic+m,1)+obsR(numStatic+m)*cos(th2), ...
         obsPos(numStatic+m,2)+obsR(numStatic+m)*sin(th2), ...
         movingColor,'FaceAlpha',0.6,'EdgeColor','k');
end

hP = plot(path(:,1),path(:,2),'b-','LineWidth',2);
plot(startPosition(1),startPosition(2),'go','MarkerFaceColor','g','MarkerSize',10);
plot(goal(1),goal(2),'p','MarkerFaceColor','y','MarkerEdgeColor','k','MarkerSize',15);
legend(hP,'Robot path','Location','best');
hold off;

%% 8. FORCE, CLEARANCE AND GOAL-DISTANCE ANALYSIS

tVec = (1:numel(FattHist))*dt;

figForce = figure('Name','Force and Clearance Analysis','Color','w', ...
    'Position',[80 60 900 750]);

subplot(3,1,1); hold on; grid on;
plot(tVec,FattHist,'Color',cAtt,'LineWidth',1.5);
plot(tVec,FrepHist,'Color',cRep,'LineWidth',1.5);
plot(tVec,FtanHist,'Color',cTan,'LineWidth',1.5);
plot(tVec,FtotHist,'Color',cTot,'LineWidth',2);
ylabel('Force magnitude');
title('Attractive / repulsive / tangential / resultant force');
legend('Attractive','Repulsive','Tangential','Resultant','Location','best');

subplot(3,1,2); hold on; grid on;
plot(tVec,clearHist,'b','LineWidth',1.5);
yline(0,'r--','Collision');
yline(d0,'k:','Influence distance d_0');
ylabel('Clearance [units]');
title('Distance to nearest obstacle surface');

subplot(3,1,3); hold on; grid on;
plot(tVec,goalDistHist,'LineWidth',1.5);
xlabel('Time [s]'); ylabel('Distance to goal [units]');
title('Distance to goal');

%% 9. POTENTIAL FIELD MAP AND ALL FORCE COMPONENTS
% The scalar potential surface is the classical attractive + repulsive map.
% Tangential steering is non-conservative, so it is shown separately as a
% vector field. Predicted-position repulsion for moving obstacles is also
% calculated and visualized separately. The resultant includes all terms.

[Xp,Yp] = meshgrid(linspace(0,10,80));
Uatt = 0.5*Katt*((Xp-goal(1)).^2+(Yp-goal(2)).^2);
Urep = zeros(size(Xp));

for i = 1:numObstacles
    rhoG = sqrt((Xp-obsPos(i,1)).^2+(Yp-obsPos(i,2)).^2) ...
           -obsR(i)-robotRadius;
    rhoG = max(rhoG,0.15);
    Urep = Urep + (rhoG < d0).*0.5*Krep.*(1./rhoG-1/d0).^2;
end

Utot = min(Uatt+Urep,40); % scalar potential: attractive + current-position repulsion
Upath = interp2(Xp,Yp,Utot,path(:,1),path(:,2)) + 0.5;

% Coarse grid for displaying each navigation force component.
[Xq,Yq] = meshgrid(0.5:0.5:9.5);
FxAtt = Katt*(goal(1)-Xq);
FyAtt = Katt*(goal(2)-Yq);
FxRep = zeros(size(Xq)); FyRep = zeros(size(Yq));
FxTan = zeros(size(Xq)); FyTan = zeros(size(Yq));
FxPred = zeros(size(Xq)); FyPred = zeros(size(Yq));

for i = 1:numObstacles
    % Repulsion from each obstacle's current position.
    dx = Xq-obsPos(i,1);
    dy = Yq-obsPos(i,2);
    dc = max(hypot(dx,dy),1e-8);
    rhoG = dc-obsR(i)-robotRadius;
    safeRho = max(rhoG,0.15);
    mag = Krep*(1./safeRho-1/d0)./(safeRho.^2);
    mag = min(max(mag,0),3.5).*(rhoG < d0);
    FxRep = FxRep + mag.*dx./dc;
    FyRep = FyRep + mag.*dy./dc;

    % Tangential steering from current obstacle position.
    tx = -dy./dc; ty = dx./dc;
    flip = tx.*(goal(1)-Xq) + ty.*(goal(2)-Yq) < 0;
    tx(flip) = -tx(flip); ty(flip) = -ty(flip);
    tanStrength = 0.6*max(0,(d0-rhoG)/d0).*(rhoG < d0);
    FxTan = FxTan + tanStrength.*tx;
    FyTan = FyTan + tanStrength.*ty;

    % For moving obstacles, add the same weighted predicted-position terms
    % used by the live controller. Keep predicted repulsion separately visible.
    if isMoving(i)
        predCenter = obsPos(i,:) + obsVel(i,:)*tPredict;
        pdx = Xq-predCenter(1);
        pdy = Yq-predCenter(2);
        pdc = max(hypot(pdx,pdy),1e-8);
        prho = pdc-obsR(i)-robotRadius;
        psafeRho = max(prho,0.15);
        pmag = Krep*(1./psafeRho-1/d0)./(psafeRho.^2);
        pmag = wPredict*min(max(pmag,0),3.5).*(prho < d0);
        FxPred = FxPred + pmag.*pdx./pdc;
        FyPred = FyPred + pmag.*pdy./pdc;

        % Predicted-position tangential steering is part of total tangent.
        ptx = -pdy./pdc; pty = pdx./pdc;
        pflip = ptx.*(goal(1)-Xq) + pty.*(goal(2)-Yq) < 0;
        ptx(pflip) = -ptx(pflip); pty(pflip) = -pty(pflip);
        ptanStrength = wPredict*0.6*max(0,(d0-prho)/d0).*(prho < d0);
        FxTan = FxTan + ptanStrength.*ptx;
        FyTan = FyTan + ptanStrength.*pty;
    end
end

% Same goal fade as the live controller (section 5c): obstacle forces
% vanish as the robot approaches the goal.
gridFade = min(1,hypot(Xq-goal(1),Yq-goal(2))/goalFadeRadius);
FxRep  = gridFade.*FxRep;  FyRep  = gridFade.*FyRep;
FxPred = gridFade.*FxPred; FyPred = gridFade.*FyPred;
FxTan  = gridFade.*FxTan;  FyTan  = gridFade.*FyTan;

% Complete grid resultant: attractive + current repulsive + predicted
% moving-obstacle repulsion + tangential (current and predicted) forces.
FxTotal = FxAtt + FxRep + FxPred + FxTan;
FyTotal = FyAtt + FyRep + FyPred + FyTan;

% Normalize vectors for readable direction plots; force magnitudes remain
% available in the live and final analysis graphs above.
nAtt = max(hypot(FxAtt,FyAtt),1e-8);
nRep = max(hypot(FxRep,FyRep),1e-8);
nTan = max(hypot(FxTan,FyTan),1e-8);
nPred = max(hypot(FxPred,FyPred),1e-8);
nTotal = max(hypot(FxTotal,FyTotal),1e-8);

figMap = figure('Name','Potential Field - All Force Components','Color','w', ...
    'Position',[40 50 1450 820]);

axM(1) = subplot(2,3,1); hold on; grid on;
surf(Xp,Yp,Utot,'EdgeColor','none','FaceAlpha',0.92);
colormap(gca,parula); colorbar;
plot3(path(:,1),path(:,2),Upath,'r-','LineWidth',2.2);
view(-40,45); xlabel('X'); ylabel('Y'); zlabel('Potential U');
title('Scalar potential (attractive + repulsive)');

axM(2) = subplot(2,3,2); hold on; axis equal; axis([0 10 0 10]); box on; grid on;
contourf(Xp,Yp,Utot,22,'LineColor','none'); colorbar;
quiver(Xq,Yq,FxAtt./nAtt,FyAtt./nAtt,0.45,'k');
title('Attractive force'); xlabel('X'); ylabel('Y');

axM(3) = subplot(2,3,3); hold on; axis equal; axis([0 10 0 10]); box on; grid on;
contourf(Xp,Yp,Utot,22,'LineColor','none'); colorbar;
quiver(Xq,Yq,FxRep./nRep,FyRep./nRep,0.45,'k');
title('Repulsive force (current positions)'); xlabel('X'); ylabel('Y');

axM(4) = subplot(2,3,4); hold on; axis equal; axis([0 10 0 10]); box on; grid on;
contourf(Xp,Yp,Utot,22,'LineColor','none'); colorbar;
quiver(Xq,Yq,FxTan./nTan,FyTan./nTan,0.45,'k');
title('Tangential steering (current + predicted)'); xlabel('X'); ylabel('Y');

axM(5) = subplot(2,3,5); hold on; axis equal; axis([0 10 0 10]); box on; grid on;
contourf(Xp,Yp,Utot,22,'LineColor','none'); colorbar;
quiver(Xq,Yq,FxPred./nPred,FyPred./nPred,0.45,'k');
title('Predicted-position repulsion (moving obstacles)'); xlabel('X'); ylabel('Y');

axM(6) = subplot(2,3,6); hold on; axis equal; axis([0 10 0 10]); box on; grid on;
contourf(Xp,Yp,Utot,22,'LineColor','none'); colorbar;
quiver(Xq,Yq,FxTotal./nTotal,FyTotal./nTotal,0.45,'w','LineWidth',0.9);
title('Resultant: all force components'); xlabel('X'); ylabel('Y');


% Obstacles (final-time positions), start, goal and path on every map panel
for p = 2:6
    axes(axM(p)); hold on;
    plot(path(:,1),path(:,2),'r-','LineWidth',1.8);
    for i = 1:numObstacles
        fill(obsPos(i,1)+obsR(i)*cos(th2),obsPos(i,2)+obsR(i)*sin(th2), ...
            [0.2 0.2 0.2],'FaceAlpha',0.8,'EdgeColor','k');
    end
    plot(startPosition(1),startPosition(2),'go','MarkerFaceColor','g','MarkerSize',8);
    plot(goal(1),goal(2),'p','MarkerFaceColor','y','MarkerEdgeColor','k','MarkerSize',12);
end

sgtitle('Potential Field and Force Components - snapshot at final time (normalized arrows show direction)');

%% 10. PERFORMANCE SUMMARY AND SAVED OUTPUTS

pathLen = sum(vecnorm(diff(path),2,2));
straightDist = norm(goal-startPosition);
timeInInfluence = sum(clearHist < d0)*dt;

fprintf('\n---------- PERFORMANCE SUMMARY ----------\n');
fprintf('Path efficiency (straight/actual): %.1f %%\n',100*straightDist/pathLen);
fprintf('Time inside obstacle influence zone: %.1f s\n',timeInInfluence);
fprintf('Time spent waiting for a moving obstacle: %.1f s\n',waitSteps*dt);
fprintf('Peak attractive force: %.2f | Peak repulsive force: %.2f\n', ...
    max(FattHist),max(FrepHist));
fprintf('Average speed: %.2f units/s\n',pathLen/(numel(tVec)*dt));

if saveOutputs
try
    T = table(tVec',FattHist',FrepHist',FtanHist',FtotHist', ...
        clearHist',goalDistHist', ...
        'VariableNames',{'time_s','F_attractive','F_repulsive', ...
        'F_tangential','F_total','clearance_m','goal_distance_m'});
    writetable(T,'simulation_log.csv');
    saveas(fig3D,'fig1_robot_3d_view.png');
    saveas(figPath,'fig2_final_path.png');
    saveas(figForce,'fig3_forces_clearance.png');
    saveas(figMap,'fig4_potential_field_map.png');
    fprintf('Saved: simulation_log.csv and 4 figure PNG files.\n');
catch ME
    fprintf('Could not save outputs: %s\n',ME.message);
end
end

drawnow;
fprintf('\n=== SIMULATION FINISHED - nothing is running now ===\n');

%% ================= Local functions =================

function setArrow(h,pos,F,k,maxLen,z)
    % Draw force F at robot position; length = k*|F| capped at maxLen
    nF = norm(F);
    if nF > 1e-8
        u = min(maxLen,k*nF)*F/nF;
    else
        u = [0 0];
    end
    set(h,'XData',pos(1),'YData',pos(2),'ZData',z, ...
          'UData',u(1),'VData',u(2),'WData',0);
end

function safe = segmentSafeMoving(p1,p2,obsPosPrev,obsPosNow,obsR,robotRadius)
    % Sample the robot segment and obstacle motion over the same time interval.
    % More samples improve detection of close approaches between time steps.
    safe = true;
    nSamples = 25;

    for k = 0:nSamples
        alpha = k/nSamples;
        robotAt = p1 + alpha*(p2-p1);
        obsAt = obsPosPrev + alpha*(obsPosNow-obsPosPrev);

        for j = 1:size(obsAt,1)
            requiredDistance = obsR(j)+robotRadius;
            if norm(robotAt-obsAt(j,:)) <= requiredDistance
                safe = false;
                return;
            end
        end
    end
end
