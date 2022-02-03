%
% two_bearing.m
%
% input:
%   pB = bearing locations (in plane) (2x2 vector, column i = pB_i)
%   phi =  bearing angle measurements (2x1 vector)
%
% output:
%   pC = center of circle of possible robot locations (2x1)
%   r = radius of the circle of possible robot locations (positive number)
%   q = start and end angles of the arc of possible robot locations (1x2)
%

function [pC,r,q1,qrange]=two_bearing(pB,phi)

% first make sure phi1-phi2 is always positive
phi12=phi(1)-phi(2);
if phi12>0
    pB1=pB(:,1);pB2=pB(:,2);
else
    pB1=pB(:,2);pB2=pB(:,1);phi12=-phi12;
end

% choose psi to be an intermediate angle of the solution range
psi=pi-phi12/2;

% unit vector from pB2 to pB1
l=norm(pB1-pB2);
e12=(pB1-pB2)/l;

% rotate vectors in the right direction
k2=rot2(psi)*e12;k1=rot2(-(pi-phi12-psi))*(-e12);

% find intersection of the two lines
alpha=inv([-k1 k2])*(pB1-pB2);
% this is one possible solution of robot location
pR=pB1+alpha(1)*k1;

% now use pB1, pB2, pC to generate the solution circle
[r,pC,k]=threepointscircle([pB1 pB2 pR;zeros(1,3)]);
pC=pC(1:2);

% find the range of angles that the robot must lie on
q(1)=atan2(pB1(2)-pC(2),pB1(1)-pC(1));
q(2)=atan2(pB2(2)-pC(2),pB2(1)-pC(1));

%if(min(sign(q))<0);q=q+2*pi;end
q=anglerange(q);
%
if q(2)<q(1);q(2)=q(2)+2*pi;end

q1=q(1);qrange=q(2)-q(1);

end

