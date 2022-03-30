%RRT path planning and generation
function path = RRT_plan(room_map,q0,qf)
    ss = stateSpaceSE2;
    sv = validatorOccupancyMap(ss);
    sv.Map = room_map;
    sv.ValidationDistance = 0.01;
    ss.StateBounds = [room_map.XWorldLimits;room_map.YWorldLimits; [-pi pi]];
    t0 = tic;
    planner = plannerRRT(ss,sv);
    tf = toc(t0);
    fprintf('RPT takes %.4f seconds to generate map\n',tf);
    planner.MaxConnectionDistance = 0.2;
    t0 = tic;
    [pthObj,solnInfo] = planner.plan(q0,qf);
    tf = toc(t0);
    fprintf('RPT takes %.4f seconds to plan the path\n',tf);
    
    rrtpath = pthObj.States(:,1:2)';
    waypoints = vecnorm(diff(rrtpath')');
    waypointslength=[0 cumsum(waypoints)];
    pathlength=sum(waypoints);
    pathvel = 1;
    ts = .5;
    lambda=(0:pathvel*ts:pathlength);
    n_l = length(lambda);
    path = zeros(2,n_l);
    path(1,:)=interp1(waypointslength,rrtpath(1,:),lambda);
    path(2,:)=interp1(waypointslength,rrtpath(2,:),lambda);
end