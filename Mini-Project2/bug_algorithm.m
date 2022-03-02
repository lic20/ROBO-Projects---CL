%Bug algorithm implementation
close all;clear all
rng('shuffle')

%get occupancy map 
colobj = roomspec();
resolution = 100;
room_map = colobj2binary_map(colobj, resolution);

rL=.4;rW=.2;rz=.1;
robot=collisionCylinder(rW,rz);
resolution = 100;

freespace_map = copy(room_map);
inflate(freespace_map, rW*1.5);

show(freespace_map);
hold on;
view(-90,90);
roomshow(colobj);

%initial location
q0 = [1;1];
robot_show(robot,q0);
%target location
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
qf = qf(1:2);
%qf = [8;8];
fprintf('Target goal is [%.2f %.2f]\n',qf(1),qf(2))

%start bug algorithm
tstart = tic;
count = 1;
q = [];
while true
    path = generate_path(q0,qf);
    [path,ind] = free_drive(robot,path,freespace_map);
    q = [q path(:,1:ind)];
    if ind == length(path)
        disp('A solution is found.')
        break
    end
    direction = randi(2); %1 = left, 2 = right
    %direction = 2;
    q0 = path(:,ind);
    %follow obstacle
    count = 1;
    while true
        if count > 2000 %make sure it doesn't go to infinite loop
            break
        end
        %check where is the collision
        subpath = generate_path(q0,qf);
        subcheck = checkOccupancy(freespace_map, subpath');
        %if we can keep going to the next obstacle border, stop following
        if check_distance(subcheck) > 12
            break
        end
        q0 = check_collision(q0,freespace_map,direction);
        if q0(1) == -1 %just to make sure robot doesn't run into the obstacle,go back a step
            if length(subpath) == 1
                q0 = subpath;
            else
                q0 = subpath(:,2);
            end
        end
        %noise
        %q0 = q0 + (rand(2,1)-0.5)*2*.1;
        robot_show(robot,q0) %show the robot path
        q = [q q0];
        count = count + 1;
    end
    
    if count > 2000
        count = -1;
        break
    end
end

if count == -1
    disp('No solution is found/algorithm got stuck')
else
    error = norm(qf-path(:,end));
    tfinal = toc(tstart);
    len = calculate_path(q);
    fprintf('The time it takes the robot to find the goal for Bug Algorithm is %.2f\n', tfinal)
    fprintf('The total length of the path is %.2f\n',len)
    fprintf('The error to the goal is %f\n',error)
end

%==================================
%Functions
%==================================

function len = calculate_path(path)
    len = 0;
    for i=1:length(path)-1
        len = len + norm(path(:,i+1)-path(:,i));
    end
end

function dist = check_distance(occup)
    dist = 0;
    for i = 1:length(occup)
        dist = dist + 1;
        if occup(i) == 1
            break
        end
    end
end

function q = check_collision(current_loc,map,direction)
    left = current_loc + [0;0.25];
    right = current_loc + [0;-0.25];
    top = current_loc + [0.25;0];
    bottom = current_loc + [-0.25;0];
    check = checkOccupancy(map, [left';right';top';bottom']);
    go_left = [0;0.15];
    go_right = [0;-0.15];
    go_up = [0.15;0];
    go_down = [-0.15;0];
    q = [-1;-1];
%     disp(check)
%     disp(current_loc)
    
    %set up different conditions
    
    if norm(check'-[1 1 1 1]) == 0
        disp('Entered the obstacle!')
        disp(current_loc)
    end
    if norm(check'-[0 0 0 0]) == 0
        loc1 = current_loc + [0.25;-0.25]; %up left
        check1 = checkOccupancy(map, loc1');
        loc2 = current_loc + [0.25;0.25]; %up right
        check2 = checkOccupancy(map, loc2');
        if check1 == 1
            q = current_loc + [0.1;-0.1];
        elseif check2 == 1
            q = current_loc + [0.1;0.1];
        end
        
        return
    end
    %3 walls
    if norm(check'-[1 1 0 1]) == 0
        q = current_loc + go_up;
        return
    end
    if norm(check'-[1 0 1 1]) == 0
        q = current_loc + go_right;
        return
    end
    if norm(check'-[0 1 1 1]) == 0
        q = current_loc + go_left;
        return
    end
    if norm(check'-[1 1 1 0]) == 0
        q = current_loc + go_down;
        return
    end
    
    %two walls
    if norm(check'-[0 1 0 1]) == 0
        if direction == 1
            q = current_loc + go_up;
        else
            q = current_loc + go_left;
        end
        return
    end
    if norm(check'-[1 0 0 1]) == 0
        if direction == 1
            q = current_loc + go_right;
        else
            q = current_loc + go_up;
        end
        return
    end
     if norm(check'-[0 1 1 0]) == 0
        if direction == 1
            q = current_loc + go_left;
        else
            q = current_loc + go_down;
        end
        return
     end
     if norm(check'-[1 0 1 0]) == 0
        if direction == 1
            q = current_loc + go_down;
        else
            q = current_loc + go_right;
        end
        return
     end
     if norm(check'-[1 1 0 0]) == 0
        if direction == 1
            q = current_loc + go_left;
        else
            q = current_loc + go_right;
        end
        return
     end
    
     %1 wall
     if check(1) == 1 
         if direction == 1
             q = current_loc + [-0.1;0];
         else
             q = current_loc + [0.1;0];
         end
     elseif check(2) == 1
         if direction == 1
             q = current_loc + [0.1;0];
         else
             q = current_loc + [-0.1;0];
         end
     elseif check(3) == 1
         if direction == 1
             q = current_loc + [0;0.1];
         else
             q = current_loc + [0;-0.1];
         end  
     elseif check(4) == 1
         if direction == 1
             q = current_loc + [0;-0.1];
         else
             q = current_loc + [0;0.1];
         end  
     end
end

function robot_show(robot,q)
    robot.Pose(1:3,4) = [q;0];
    show(robot);
end

function [path,index] = free_drive(robot,path,map)
    occup = checkOccupancy(map, path');
    index = length(path);
    if occup(1) == 1
        index = 1;
        return
    end
    for k=2:length(path)
        if occup(k) == 1
            index = k-1;
            break;
        end
        %noise
        %path(:,k) = path(:,k) + (rand(2,1)-0.5)*2*.1;
        robot_show(robot,path(:,k))
    end
end

function path = generate_path(q0,qf)
    lf = norm(qf-q0);
    N = lf*9;
    l = [0:lf/N:lf];
    path = q0*(1-l/lf) + qf*(l/lf); %equation of p
end

