% show robot in figure

function robotshow(robot,q,colorcode);

if isprop(robot,'Z')
    rz = robot.Z/2;
else
    rz = 0;
end
robot.Pose(1:3,4)=[q(1);q(2);rz/2];
robot.Pose(1:3,1:3)=rot([0;0;1],q(3));
[~,patchObj] = show(robot);
if ~exist('colorcode')
    patchObj.FaceColor = [218,165,32]/218;
else
    patchObj.FaceColor = colorcode(1:3);
end
patchObj.EdgeColor = 'none';

% show heading direction
robotleng=1;
h=quiver(q(1),q(2),robotleng*cos(q(3)),robotleng*sin(q(3)));
set(h(1),'LineWidth',3);

end

%
% rot function
%
function R=rot(k,theta)
  
  k=k/norm(k);
  R=eye(3,3)+sin(theta)*hat(k)+(1-cos(theta))*hat(k)*hat(k);
  
end

%
% hat function
%
function khat = hat(k)
  
  khat=[0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];

end