
function ray=defineray(l,r,q,rz,phi)
ray=collisionCylinder(r,l);
ray.Pose(1:3,1:3)=rot([1;0;0],pi/2);
ray.Pose(1:3,4)=[0;l/2;0];
ray.Pose(1:3,:)=rot([0;0;1],-pi/2+q(3)+phi)*ray.Pose(1:3,:);
ray.Pose(1:3,4)=ray.Pose(1:3,4)+[q(1:2);rz];
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