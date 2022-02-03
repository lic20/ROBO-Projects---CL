%
% >> autodrive_mapping
% Mapping algorithm

% input u is assume available
close all

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
pB(3,:)=[0 0];
%
N_range=size(pL,2);N_bearing=size(pB,2);N_odo=2;
%
ns=N_range+N_bearing+N_odo+N_scan; % total # of sensors
wcov=[0.05;0.05];vcov=.05*ones(ns,1); % noise covariance
% steering command and sampling period
ts=1;
% initial sensor reading
y = [];
y(:,1)=output(q(:,1),pL,pB,N_scan,[0;0],vcov,robot,colobj);
% initial landmark estimates
N_param=3*(N_range+N_bearing);sigma_a=.5;
atrue=[reshape(pL,3*N_range,1);reshape(pB,3*N_bearing,1)];
ahat(:,1)=atrue+randn(N_param,1)*sigma_a;
P{1}=sigma_a^2*eye(N_param,N_param);S{1}=sigma_a^2*eye(N_param,N_param);

N_step=size(u,2);
%N_step=50;

for k=1:N_step
    % use true state to generate sensor output    
    y(:,k+1)=output(q(:,k+1),pL,pB,N_scan,utrue(:,k),vcov,robot,colobj);
    % EKF propagation of landmark locations
    [ahat(:,k+1),P{k+1},S{k+1}]=...
        landmark_est(ahat(:,k),q(:,k),u(:,k),y(:,k), ...
        N_range,N_bearing,N_odo,vcov,P{k},S{k},robot,colobj);
    robotshow(robot,q(:,k+1));
    k=k+1;
end

% post-run analysis

t=[0:N_step];

figure(2);plot(t,atrue-ahat,'linewidth',2);

disp('Landmark Parameter Comparisons')
disp('True, Final Value, Staring Value')
disp([atrue ahat(:,N_step) ahat(:,1)])
