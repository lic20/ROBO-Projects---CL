%
% simulate sensors
%

% range sensors from UWB sensors

function y=output_simple(q,utrue,vcov,robot,colobj)

% height of the sensor on robot
rz=robot.Z/2;
ez=[0;0;1];

% position and orientation

pR=[q(1:2);rz];
theta=q(3);

% UWB (local GPS)

zW=colobj.obj{1}.Z;
lW=colobj.obj{1}.X;
zW=0;
pL(:,1)=[0;0;zW];pL(:,2)=[lW;0;zW];pL(:,3)=[0;lW;zW];pL(:,4)=[lW;lW;zW];

for i=1:size(pL,2)
    y(i)=norm(pR-pL(:,i));
end

% odometry and IMU

y(5:6)=utrue;
    
% bearing sensor to cylinder

% how to check if view is obstructed?

pB(:,1)=colobj.obj{14}.Pose(1:3,4);
y(7)=atan2(pB(2,1)-q(2),pB(1,1)-q(1))-theta;
pB2(:,1) = colobj.obj{10}.Pose(1:3,4);
y(8)= atan2(pB2(2,1)-q(2),pB2(1,1)-q(1))-theta;

% scanner 
M=8; % # of scan lines
ang=2*pi*[0:M-1]/M;
for i=1:M
    % unit vector in each scan direction
    y(8+i)=rayscan(robot,q,colobj,ang(i));
end

y=y';
l=length(y);
v=zeros(size(y));
v=normrnd(v,vcov(1:l));
y=y+v;
end