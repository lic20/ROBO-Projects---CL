%
% two_bearing_one_range.m
%
% input:
%   pB = bearing locations (in plane) (2x2 vector, column i = pB_i)
%   pL = range marker location (3x1 vector)
%   phi =  bearing angle measurements (2x1 vector)
%   d = range sensor measurement (scalar)
%
% output:
%   pR = solution of robot location 
%   qR = robot orientation angle
%

function [pR,qR]=two_bearing_one_range(pB,pL,phi,d)

% use two bearing sensors to find the soution circles
[pC,r,qA,qA]=two_bearings(pB(1:2,1:2),phi(1:2));

% || pC + rot(ez,q)*r*ex - pL || = d
ex=[1;0;0];ez=[0;0;1];
q = subprob3(ez,ex*r,pL-[pC;0],d);

for i=1:length(q)
    pR(:,i) = [pC;0]+rot(ez,q(i))*r*ex;
    qR(i)=subprob1(ez,ex,[rot2(-phi(1))*(pB(1:2,1)-pR(1:2,i))...
        /norm(pB(1:2,1)-pR(1:2,i));0]);
end

end
