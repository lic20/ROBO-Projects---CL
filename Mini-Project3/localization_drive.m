%using repellent potential with perfect localization
clear all; close all

%choose a number of agents:
n = input(['Enter the number of agents ']);
n=n+~mod(n,2);

%conduct two cases: n agents in a ring or fully connected graph
selection = input(['Enter Mode number (1 or 2) \n',...
    '1: v-shaped\n', '2:circle formation \n']);

switch selection
    case 1
        % set up the incidence matrix for V graph
        D=zeros(n,n-1);
        D(1,1)=-1;D(1,1+(n-1)/2)=-1;
        D((n-1)/2+1,(n-1)/2)=1;D(n,(n-1))=1;
        for i=2:(n-1)/2
            D(i,i-1)=1;D(i,i)=-1;
            D(i+(n-1)/2,(n-1)/2+i-1)=1;D(i+(n-1)/2,(n-1)/2+i)=-1;
        end
        % set up the target difference variables
        zstar1 = [-.3;-.3];
        zstar2 = [.3;-.3];
        zstar = [kron(ones((n-1)/2,1),zstar1);kron(ones((n-1)/2,1),zstar2)];
    % 2 = circle formation 
    otherwise
        % set up incidence matrix for a ring graph
        D=zeros(n,n);
        D(1,1)=-1;D(1,n)=1;
        D(n,n-1)=1;D(n,n)=-1;
        for i=2:n-1
            D(i,i-1)=1;D(i,i)=-1;
        end
        % set up target difference variables 
        zc = .6;
        zstar = zeros(2*n,1);
        zstar(1:2)=zc*[cos(pi/n);-sin(pi/n)];
        for i=2:n
            zstar((i-1)*2+1:2*i)=...
                zc*[cos((i-1)*2*pi/n+pi/n);-sin((i-1)*2*pi/n+pi/n)];
        end
end
%create graph
G=graph(-D*D'+diag(diag(D*D')));
figure(5);plot(G);
set(gcf,'position',[100,400,400,400]);

%show the room:
colobj = roomspec();
resolution = 100;
room_map = colobj2binary_map(colobj, resolution);

%virtual robot for path planning
rL=.6;rW=.5;rz=.1;
robot=collisionCylinder(rW,rL);

inflate(room_map, 0.25);
roomshow(colobj);

if ~exist('qf')
    qf_flag = 1;
    while qf_flag>0
        qf = [5*rand(1,2)+[5 5] 0]';
        [isInt,dist,wp]=colcheck(robot,qf,colobj);
        if (max(isnan(dist))==0) && (min(dist)>0.5)
            qf_flag = 0;
        end
    end
end
%qf = [6.52;5.7;0];
%qf = [9;5;0];
fprintf('Target goal is [%.2f %.2f]\n',qf(1),qf(2))
%RPT plannar
path = RRT_plan(room_map,[1 1 0],qf');
plot(path(1,:),path(2,:),'x-')

Dfull=(incidence(graph(ones(n,n)-diag(ones(n,1)))));


N = length(path);
l = size(D,2);

q = zeros(2*n,N+1);
u = zeros(2*n,N);
z = zeros(2*l,N+1);

%localization parameters
zW=colobj.obj{1}.Z;
lW=colobj.obj{1}.X;
zW=0;
pL(:,1)=[0;0;zW];pL(:,2)=[lW;0;zW];pL(:,3)=[0;lW;zW];pL(:,4)=[lW;lW;zW];
% **** bearing sensors ****
pB(:,1)=colobj.obj{14}.Pose(1:3,4);
pB(:,2)=colobj.obj{10}.Pose(1:3,4);
%
vcov=0.1*ones(6,1); % noise covariance
P = 0.05*eye(2);
Sigma = 0.05*eye(2);

%initialize first variables:
q(:,1) = rand(2*n,1);
z(:,1) = kron(D',eye(2))*q(:,1);

%time
ts = 1;
t = (0:N)*ts;

Kp1 = 0.1*eye(2*n); %gain for repel robots to certain distance
Kp2 = 0.15*eye(2*n); %gain for keep robots in the formation

%potential function
qrange=.05;
psifun = @(x,dx) (x<dx).*(1./x-1./dx);

Kp_lead = 0.1; %lead agent feed forward gain
test_robot=collisionCylinder(0.2,0.2);

t0 = tic;
for k=1:N
    %compute the velocity of the next step
    if k < N
        vel = kron(ones(n,1),path(:,k+1)-path(:,k));
    else
        vel = zeros(size(vel));
    end
    
    %relative distance to keep formation
    zk = kron(D',eye(2))*q(:,k);
    phi = Kp2*kron(D,eye(2))*(zk-zstar);
    
    %repellent force to keep robots apart
    zkfull = kron(Dfull',eye(2))*q(:,k);
    zkflat = reshape(zkfull,2,size(Dfull,2));
    distzk = vecnorm(zkflat);
    
    psiflat = zkflat.*psifun(distzk,qrange)./distzk;
    psi = Kp1*kron(Dfull,eye(2))*reshape(psiflat,2*size(Dfull,2),1);
    
    %update variables:
    u(:,k) = -psi - phi + vel;
    %lead agent implementation:
    %u(1:2,k) = u(1:2,k) - Kp_lead*(q(1:2,k)-path(:,k));
    %distributed control implementation:
    u(:,k) = u(:,k) + kron(ones(n,1),-Kp_lead*(q(1:2,k)-path(:,k)));
    
    %q(:,k+1) = q(:,k) + ts*u(:,k);
    %apply localization
    [q(:,k+1),P,Sigma] = localization(n,q(:,k),vcov,pL,pB,u(:,k),P,Sigma);
    if k == N
        qf = path(:,k);
    else
        qf = path(:,k+1);
    end
    q(:,k+1) = object_avoidance(n,q(:,k+1),test_robot,colobj,qf);
    z(:,k+1) = kron(D',eye(2))*q(:,k+1);
end
tfinal = toc(t0);
fprintf('The time it takes to compute all the robot paths is %.2f\n',tfinal)

%plot robots
for k=1:N+1
    qk=q(:,k);
    qkflat=reshape(qk,2,n);
    plot(qkflat(1,:),qkflat(2,:),'o','linewidth',3);
    M(k)=getframe;
end
%movie(M)
write_video(M,selection)

figure(10);hold on
%plot formation difference
dz = z-zstar;
for i=1:l
    plot(t,vecnorm(dz((i-1)*2+1:i*2,:)),'linewidth',2);
    xlabel('time (s)');ylabel('formation difference error (m)');
end

function [q_new, P, Sigma] = localization(n,q,vcov,pL,pB,u,P,Sigma)
    qflat = reshape(q,2,n);
    uflat = reshape(u,2,n);
    q_new = [];
    for i=1:length(qflat)
        y = output(qflat(:,i),pL,pB,vcov);
        [qhat_next,P,Sigma] = pose_est_filter(qflat(:,i),uflat(:,i),y,pL,pB,vcov,P,Sigma);
        q_new = [q_new; qhat_next];
    end
end

%write the movie to a local file
function write_video(M,mode)
    %write movie to file:
    filename = 'localization_drive_v.avi';
    if mode == 2
        filename = 'localization_drive_circle.avi';
    end
    v = VideoWriter(filename);
    v.FrameRate = 10; %slow down the playback of the video
    open(v)
    writeVideo(v, M)
    close(v)
end