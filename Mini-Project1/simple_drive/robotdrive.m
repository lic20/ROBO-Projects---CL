%
% use keyboard to drive a wheeled mobile robot
% 
% to run: just type in 
% 
% >> robotdrive
% 
% use the arrow key in the figure window to drive the robot 
% (up=forward, down=backware, left=turn left, right=turn right)
% press q to quit
%
% this program calls 
%
% roomspec: set up the room and all collision objects in colobj structure
% roomshow: show all the collision objects
% robotspec: set up the robot collision body
% robotshow: show robot in the same plot
% robotfcn: function that gets called when a key is pressed in the figure
% output: generates all the sensor reading based on the current robot pose
% pose_est: estimating the pose of the robot
% colcheck: collision check between robot and all objects in room
%

clear all;close all;
% define room
roomspec;
% show room
fignum=1;
h_fig=roomshow(colobj,fignum);
axis('square');
% define robot
rL=.4;rW=.2;rz=.1;
robot=robotspec([rL;rW;2*rz]);
% show robot
q0=[.2;2;0];
q(:,1)=q0;
robotshow(robot,q0);
%
ez=[0;0;1];
% # of scan lines in lidar
N_scan=8; 
% **** range sensors ****
% UWB (local GPS)
zW=colobj.obj{1}.Z;
lW=colobj.obj{1}.X;
zW=0;
pL(:,1)=[0;0;zW];pL(:,2)=[lW;0;zW];pL(:,3)=[0;lW;zW];pL(:,4)=[lW;lW;zW];
% **** bearing sensors ****
pB(:,1)=colobj.obj{14}.Pose(1:3,4);
pB(:,2)=colobj.obj{10}.Pose(1:3,4);
ns=8+N_scan; % total # of sensors
wcov=[0.05;0.05];vcov=.1*ones(ns,1); % noise covariance
% steering command and sampling period
v=.1;w=.1;ts=1;
% initial sensor reading
y(:,1)=output(q(:,1),pL,pB,N_scan,[0;0],vcov,robot,colobj);
% initial state estimate
qhat(:,1)=pose_est(y(:,1),pL,pB,N_scan,wcov,vcov);

k=1;
% set up key press detection in figure
set(h_fig,'KeyPressFcn',@robotfcn);
keyvalue='a';
keypressed=0;

% keypress of q will quit
while keyvalue~='q'
    while keypressed==0;pause(.05);end
    u(:,k)=[0;0];
    keypressed=0;
    switch keyvalue
        case 'uparrow'
            u(1,k)=v;
        case 'downarrow'
            u(1,k)=-v;
        case 'leftarrow'
            u(2,k)=w;
        case 'rightarrow'
            u(2,k)=-w;
    end
    % propagate robot state
    [q(:,k+1),utrue(:,k)]=wmr(q(:,k),u(:,k),ts,wcov);
    % generate sensor output    
    y(:,k+1)=output(q(:,k+1),pL,pB,N_scan,utrue(:,k),vcov,robot,colobj);
    % estimate robot state
    qhat(:,k+1)=pose_est(y(:,k+1),pL,pB,N_scan,wcov,vcov);
    % check collision 
    [isInt,dist,wp]=colcheck(robot,q(:,k+1),colobj);   
    if max(isnan(dist))>0
        q(:,k+1)=q(:,k);disp('collision!');
    end
    robotshow(robot,q(:,k+1));
    k=k+1;
end

% post-run analysis

t=[1:k];
figure(2);plot(t,q,t,qhat,':','linewidth',2);
legend('x','y','\theta','x_{est}','y_{est}','\theta_{est}');