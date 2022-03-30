%formation control + path planning with perfect localization
clear all; close all

%choose a number of agents:
n = input(['Enter the number of agents ']);
if n <=0
    n = fix((rand+.5)*20);
end

%conduct two cases: n agents in a ring or fully connected graph
selection = input(['Enter Mode number (1 or 2) \n',...
    '1: ring configuration\n', '2: fully connected \n']);

%show the room:
colobj = roomspec();
resolution = 100;
room_map = colobj2binary_map(colobj, resolution);

%virtual robot for path planning
rL=.8;rW=.5;rz=.1;
robot=collisionCylinder(rW,rL);

inflate(room_map, 0.25);

switch selection
    case 1
        n1=(1:n);
        n2=[(2:n) 1];
    case 2
        n1=[];n2=[];
        for i=1:n
            n1=[n1 i*ones(1,n-i)];
            n2=[n2 (i+1:n)];
        end
end

%create graph
G=graph(n1,n2);
D=full(incidence(G));

figure(2);plot(G);
set(gcf,'position',[100,400,500,500])

% figure();show(room_map);
% hold on;
% view(-90,90);
roomshow(colobj);

l = size(D,2);
states = 2;
N = 100;

total_q = [];
total_z = [];
%randomly generate initial condition
q0 = rand(states*n,1);
z0 = kron(D',eye(2))*q0;

dc = 1;
dz = dc*2*pi/n*ones(l,1);

zstar = dz(1)*ones(2*l,1);
% feedback gain
Kp = .2*eye(n*states,n*states);
Kp1 = .4*eye(n*states,n*states);

%potential function
s = @(x,d) log(x/d);

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
%qf = [9;5;0];
fprintf('Target goal is [%.2f %.2f]\n',qf(1),qf(2))

%RPT plannar
path = RRT_plan(room_map,[1 1 0],qf');
plot(path(1,:),path(2,:),'x-')

current_loc = q0;
t0 = tic;
for i=1:length(path)
    center = path(:,i);
    [new_q,new_z] = formation(center,n,N,D,l,current_loc,dc,dz,s,Kp,Kp1,colobj);
    current_loc = new_q(:,end);
    total_q = [total_q new_q(:,2:end)];
    total_z = [total_z new_z(:,2:end)];
end
tfinal = toc(t0);
fprintf('The time it takes to compute all the robot paths is %.2f\n',tfinal)

%plot robots
for k=1:length(total_q)
    qk=total_q(:,k);
    qkflat=reshape(qk,states,n);
    plot(qkflat(1,:),qkflat(2,:),'o','linewidth',3);
    M(k)=getframe;
end
%movie(M);
write_video(M,selection)

diffz = abs(total_z) -  dz(1)*ones(2*l,1);
figure(7);hold on
for i=1:l
    plot(vecnorm(diffz((i-1)*2+1:i*2,:)),'linewidth',2);
    xlabel('iterations');ylabel('formation difference error (m)');
    title('Error For Distance Between Links')
end
hold off

function write_video(M,mode)
    %write movie to file:
    filename = 'simple_drive_ring.avi';
    if mode == 2
        filename = 'simple_drive_circle.avi';
    end
    v = VideoWriter(filename);
    v.FrameRate = 10; %slow down the playback of the video
    open(v)
    writeVideo(v, M)
    close(v)
end

function [return_q,return_z] = formation(center,n,N,D,l,q0,dc,dz,s,Kp,Kp1,colobj)
    %initialization
    q = zeros(2*n,N+1);
    u = zeros(2*n,N);
    z = zeros(2*l,N+1);
    
    q(:,1)= q0;
    z(:,1) = kron(D',eye(2))*q(:,1);
    
    test_robot=collisionCylinder(0.2,0.2);
    
    return_q = [];
    return_z = [];
    for k=1:N
        % link difference vectors z = D^T * q 
        %disp(q(:,k))
        zk = kron(D',eye(2,2))*q(:,k);
        % for each link compute the potential gradient
        for j=1:l
            zkj = zk((j-1)*2+1:j*2);        
            psi((j-1)*2+1:j*2,1) = s(norm(zkj),dz(j))*zkj/norm(zkj);
        end
        % potential gradient to the origin
        for i=1:n
            qki = q((i-1)*2+1:i*2,k);
            phi((i-1)*2+1:i*2,1) = s(norm(qki-center),dc)*(qki-center)/norm(qki-center);
        end
        % controller combines the two
        u(:,k) = -Kp*kron(D,eye(2,2))*psi - Kp1 * phi;
        q(:,k+1) = q(:,k)+u(:,k);
        q(:,k+1) = object_avoidance(n,q(:,k+1),test_robot,colobj,center);
        z(:,k+1) = kron(D',eye(2))*q(:,k+1);
        if mod(k,20) == 0 || k == 1
            return_q = [return_q q(:,k+1)];
            return_z = [return_z z(:,k+1)];
        end
    end
end
