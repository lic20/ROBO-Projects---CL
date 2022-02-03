%
% threepointscircle: fits 3 points to a circle
% 
% input: 
% L = [x1 x2 x3]
%     [y1 y2 y3]
%     [z1 z2 z3]
%
% output:
% r = radius
% po = center of the circle
% k = vector perpendicular to the plane containing the three points
% 

% Denote columns L by [p1 p2 p3]
% Use SSS to find the correspoding angles q1 q2 q3
% look at the two triangles formed by pC p1 p2 and by pC p2 p3
% Let the C23 angle be e2, C12 angle be e2, C13 angle be e3
% then
% q1=e2-e3
% q2=e1+e2
% q3=e1-e3
%
% solve for [e1 e2 e3]
% from the triangle C12, we have ||p2-p1||^2 = 2r^2 -2r^2 cos(pi-2*e2)
% solve for r
%
% find the perpendicular axis k = (p2-p1)x(p3-p1) (then normalize)
%
% pC = rot(k,e2)* r * normalized(p2-p1) + p1 
%

function [r,po,k]=threepointscircle(L)

p1=L(:,1);
p2=L(:,2);
p3=L(:,3);

q=SSS(L);

e=inv([0 1 -1;1 1 0;1 0 -1])*q;

r=sqrt(0.5*norm(p2-p1)^2/(1-cos(pi-2*e(2))));
k=cross(p2-p1,p3-p1);k=k/norm(k);
po=r*rot(k,e(2))*(p2-p1)/norm(p2-p1)+p1;

end