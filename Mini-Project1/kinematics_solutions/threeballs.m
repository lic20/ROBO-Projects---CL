%
% threeballs.m
% 
% find intersections of three balls with centers at p1,p2,p3 
%   and radii r1, r2, r3
% returns up to two solutions 
% 
function z=threeballs(p1,p2,p3,r1,r2,r3)

k=(p2-p1)/norm(p2-p1);

kperp=cross(k,p1)/norm(cross(k,p1));
q1=subprob3(kperp,r1*k,p2-p1,r2);
r1rot=rot(kperp,q1(1))*k*r1;

q=subprob3(k,r1rot,p3-p1,r3);

z(:,1)=rot(k,q(1))*r1rot+p1;
z(:,2)=rot(k,q(2))*r1rot+p1;

end
