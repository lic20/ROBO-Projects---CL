%
% two_range_sensor_and_plane
%
% find the (max) two possible solutions of intersections of two balls
% (described by center location p1,p2 and radii r1, r2) and a plane
% (descirbed by the unit normal vector, k, and any point on the plane, p0)
%
% z=two_range_sensor_and_plane(p1,p2,r1,r2,k,p0)
%

function z=two_range_sensor_and_plane(p1,p2,r1,r2,k,p0)

% first find the circles defining the intersection of two spheres with the
% plane
% ** first radii
d1 = sqrt(r1^2-((p1-p0)'*k)^2);
d2 = sqrt(r2^2-((p2-p0)'*k)^2);
% then centers 
pc1 = p1 - ((p1-p0)'*k)*k;
pc2 = p2 - ((p2-p0)'*k)*k;

% find the intersections two circles
q1 = subprob3(k,(pc2-pc1)/norm(pc2-pc1)*d1,pc2-pc1,d2);

% calculate the solution vector 

z(:,1) = pc1+rot(k,q1(1))*(pc2-pc1)/norm(pc2-pc1)*d1;
z(:,2) = pc1+rot(k,q1(2))*(pc2-pc1)/norm(pc2-pc1)*d1;

end
