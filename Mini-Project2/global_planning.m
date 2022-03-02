%PRM
close all;clear all
rng(100,'twister')
mode = 0; %choose a method: 0 = PRM, 1 = RPT

%get occupancy map 
colobj = roomspec();
resolution = 100;
room_map = colobj2binary_map(colobj, resolution);

rL=.4;rW=.2;rz=.1;
robot=collisionCylinder(rW,rL);
resolution = 100;

freespace_map = copy(room_map);
inflate(freespace_map, rW);

show(freespace_map);
hold on;
view(-90,90);
roomshow(colobj);

%initial location
q0 = [1;1;0];
robot_show(robot,q0);

%get target position
if ~exist('qf')
    qf_flag = 1;
    while qf_flag>0
        qf = [5*rand(1,2)+[5 5] 0]';
        [isInt,dist,wp]=colcheck(robot,qf,colobj);
        if (max(isnan(dist))==0) && (min(dist)>0.5)
            qf_flag = 0;
        end
    end
end
%qf(1:2) = [8;8];
fprintf('Target goal is [%.2f %.2f]\n',qf(1),qf(2))

if mode == 0
    % %prm plannar
    t0 = tic;
    plannar = mobileRobotPRM(freespace_map,250);
    tf = toc(t0);
    fprintf('PRM takes %.4f seconds to generate map\n',tf);
    t0 = tic;
    path = findpath(plannar,q0(1:2)',qf(1:2)');
    tf = toc(t0);
    fprintf('PRM takes %.4f seconds to plan the path\n',tf);
    show(plannar)
    len = calculate_path(path');
    fprintf('The total length of the path is %.2f\n',len)
else
    %RPT plannar
    ss = stateSpaceSE2;
    sv = validatorOccupancyMap(ss);
    sv.Map = freespace_map;
    sv.ValidationDistance = 0.01;
    ss.StateBounds = [freespace_map.XWorldLimits;freespace_map.YWorldLimits; [-pi pi]];
    t0 = tic;
    planner = plannerRRT(ss,sv);
    tf = toc(t0);
    fprintf('RPT takes %.4f seconds to generate map\n',tf);
    planner.MaxConnectionDistance = 0.2;
    t0 = tic;
    [pthObj,solnInfo] = planner.plan(q0',qf');
    tf = toc(t0);
    fprintf('RPT takes %.4f seconds to plan the path\n',tf);
    len = calculate_path(pthObj.States(:,1:2)');
    fprintf('The total length of the path is %.2f\n',len)
    
    plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-'); % tree expansion
    plot(pthObj.States(:,1), pthObj.States(:,2),'r-','LineWidth',2) % draw path
    
end


function len = calculate_path(path)
    len = 0;
    for i=1:length(path)-1
        len = len + norm(path(:,i+1)-path(:,i));
    end
end

function robot_show(robot,q)
    robot.Pose(1:3,4) = q;
    show(robot);
end