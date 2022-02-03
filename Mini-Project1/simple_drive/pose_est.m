%
% pose_est.m
%
% Given y and robot kinematics, estimate the robot position and orientation
%
function qhat=pose_est(y,pL,pB,N_scan,wcov,vcov)

rz=.1;ez=[0;0;1];

% use two range sensors intersecting with the x-y plane
i=1;j=2;

kvec=(pL(:,i)-pL(:,j))/norm(pL(:,i)-pL(:,j));
d1=sqrt(y(i)^2-(pL(:,i)'*ez-rz)^2);
d2=sqrt(y(j)^2-(pL(:,j)'*ez-rz)^2);
phi=subprob3(ez,d1*kvec,pL(:,j)-pL(:,i),d2);
pp1=pL(:,i)-pL(:,i)'*ez*ez;
qest=[pp1+rot(ez,phi(1))*d1*kvec+rz*ez pp1+rot(ez,-phi(1))*d1*kvec+rz*ez];

% use three balls intersection to disambiguate
i=1;j=2;k=3;
qest1=threeballs(pL(:,i),pL(:,j),pL(:,k),y(i),y(j),y(k));

[dum,ind]= min(vecnorm(qest(1:2,:)-mean(qest1(1:2,:),2)));
qhat=qest(:,ind);


% assume a bearing sensor
pBi= pB(:,1);

qhat(3,1)=atan2(pBi(2,1)-qhat(2,1),pBi(1,1)-qhat(1,1))-y(7);
end