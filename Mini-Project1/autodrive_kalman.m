%
% autodrive the robot with a set path
%
% this program calls 
%
% roomspec: set up the room and all collision objects in colobj structure
% roomshow: show all the collision objects
% robotspec: set up the robot collision body
% robotshow: show robot in the same plot
% robotfcn: function that gets called when a key is pressed in the figure
% output: generates all the sensor reading based on the current robot pose
% pose_est_filter: estimating the pose of the robot
% colcheck: collision check between robot and all objects in room
%
close all
% input u is assume available

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
%
N_range=size(pL,2);N_bearing=size(pB,2);N_odo=2;
%
ns=N_range+N_bearing+N_odo+N_scan; % total # of sensors
wcov=[0.05;0.05];vcov=.1*ones(ns,1); % noise covariance
% steering command and sampling period
v=.1;w=.1;ts=1;
% initial sensor reading
y(:,1)=output(q(:,1),pL,pB,N_scan,[0;0],vcov,robot,colobj);
% initial state estimate
qhat(:,1)=pose_est(y(:,1),pL,pB,N_scan,wcov,vcov);

% initialization for the EKF
qhat_EKF(:,1)=qhat(:,1);P{1}=.1*eye(3,3);S{1}=.1*eye(3,3);

N_step=size(u,2);
%N_step=50;

for k=1:N_step
    %disp(k)
    % propagate robot state
    % [q(:,k+1),utrue(:,k)]=wmr(q(:,k),u(:,k),ts,zeros(size(wcov)));
    % generate sensor output    
    y(:,k+1)=output(q(:,k+1),pL,pB,N_scan,utrue(:,k),vcov,robot,colobj);
    % estimate robot state
    qhat(:,k+1)=pose_est(y(:,k+1),pL,pB,N_scan,wcov,vcov);
    % EKF estimate
    [qhat_EKF(:,k+1),P{k+1},S{k+1}]=... 
        pose_est_filter(qhat_EKF(:,k),u(:,k),y(:,k),ts,...
        pL,pB,N_scan,wcov,vcov,P{k},S{k},robot,colobj);
    %[qhat_iter,delta_y]=pose_est_NLS(qhat(:,k+1),y(:,k+1),pL,pB);    
    %dy{k}=delta_y;
    %qhat1(:,k+1)=qhat_iter(:,size(qhat_iter,2));    
    % check collision 
    [isInt,dist,wp]=colcheck(robot,q(:,k+1),colobj);   
    if max(isnan(dist))>0
        q(:,k+1)=q(:,k);disp('collision!');
    end
    robotshow(robot,q(:,k+1));
    k=k+1;
end

% post-run analysis

t=[0:N_step];
figure(2);h=plot(t,q(:,1:N_step+1),t,qhat(:,1:N_step+1),'-',...
    t,qhat_EKF(:,1:N_step+1),':');
set(h(1),'linewidth',2);set(h(2),'linewidth',2);set(h(3),'linewidth',2);
set(h(4),'linewidth',1);set(h(5),'linewidth',1);set(h(6),'linewidth',1);
set(h(7),'linewidth',2);set(h(8),'linewidth',2);set(h(9),'linewidth',2);
legend('x','y','\theta','x_{est}','y_{est}','\theta_{est}',...
    'x_{EKF}','y_{EKF}','\theta_{EKF}');

figure(3);plot(t(2:end),vecnorm(q(:,2:N_step+1)-qhat(:,2:N_step+1)),'--x',...
    t(2:end),vecnorm(q(:,2:N_step+1)-qhat_EKF(:,2:N_step+1)),':o','linewidth',2);
legend('Direct','EKF');


