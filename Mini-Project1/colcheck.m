%
% colcheck: check collision distance between robot at pose q and 
%           all collision objects
%
% input:
%       robot = robot collision object
%       q = robot pose
%       colobj = room collision objects
%
% output: 
%       isInt: collision indicator between robot and each collision object
%       dist: distance between robot and collision objects
%       wp: witness points between robot and collision objects
%
function [isInt,dist,wp]=colcheck(robot,q,colobj)

% get robot's height
if isprop(robot,'Z')
    rz = robot.Z/2;
else
    rz = 0;
end
% update robot collision body pose
robot.Pose(1:3,4)=[q(1:2);rz];
robot.Pose(1:3,1:3)=rot([0;0;1],q(3));

% check collision distance with all collision objects
Ncolobj=length(colobj.obj);

for i=1:Ncolobj
    [isInt(i),dist(i),wp{i}]=checkCollision(robot,colobj.obj{i});
end

end