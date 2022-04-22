%
% 2D formation control example based on the relative pose in a room
% 
clear all;close all;

% *****
% define room and robot
% *****
% define room
colobj=roomspec;
% define robot
rL=.4;rW=.2;rz=.1;
robot=collisionCylinder(rW,rz);
load_x=1;load_y=0.5;
load=collisionBox(load_x,load_y,rz);

% *****
% import desired path (assuming same sampling period ts)
% variable name of path: despath
% *****

while ~exist('despath')
    filename=input('enter desired path file name with variable despath: ','s');
    eval(['load ',filename]);
    if ~exist('despath');disp('No variable named *despath*');end
end

% # of time steps
N = length(despath);

figure(1);hold on;
roomshow(colobj,1);axis('square');
view(-90,90);axis([-1 11 -1 11 0 4]);
title('desired path')
despathdiff=diff(despath')';
theta=atan2(despathdiff(2,:),despathdiff(1,:));
despath(3,:)=[theta theta(N-1)];
for k=1:N
    robotshow(load,despath(:,k));pause(.1);
end

% trajectory generation with trapezoidal profile

lambda=[0 cumsum(vecnorm(despathdiff))];
lddotmax=.05;
ldotmax=0.5;
% lddotmax=.2;
% ldotmax=1;
[x,v,a,ta,tb,tf]=trapgen(0,lambda(end),0,0,ldotmax,lddotmax,lddotmax,0);
fprintf('tf for trapezoidal profile = %g\n',tf);
N=100;
t_trap=(0:tf/N:tf);
for i=1:length(t_trap)
  [l_x(i),l_v(i),l_a(i),ta,tb,tf]=...
      trapgen(0,lambda(end),0,0,ldotmax,lddotmax,lddotmax,t_trap(i));    
end

qdes=interp1(lambda',despath',l_x)';
figure(2);
subplot(3,1,1);plot(t_trap,l_x);
subplot(3,1,2);plot(t_trap,l_v);
subplot(3,1,3);plot(t_trap,l_a);
figure(3);
% put orientation first
qdes=qdes([3;1;2],:);
subplot(3,1,1);plot(lambda,despath(2,:),'o',l_x,qdes(1,:),'k-')
xlabel('\lambda (m)');ylabel('x(\lambda) (m)');
subplot(3,1,2);plot(lambda,despath(3,:),'o',l_x,qdes(2,:),'k-')
xlabel('\lambda (m)');ylabel('y(\lambda) (m)');
subplot(3,1,3);plot(lambda,despath(1,:),'o',l_x,qdes(3,:),'k-')
xlabel('\lambda (m)');ylabel('\theta(\lambda) (m)');
figure(4);
plot(t_trap,qdes);xlabel('time (s)');ylabel('q(t)');
legend('x','y','\theta');

%
% motion control of load 
%
Mc=10; % 10Kg
Ic=(1/12)*Mc*(load_x^2+load_y^2);    % I_zz (for 1x0.5 m)
MIc=diag([Ic;Mc;Mc]);

ts=1;
N=60;
tf=ts*N;
xc=zeros(6,N+1);
xc(:,1)=[despath(:,1)+randn(3,1)*.1;zeros(3,1)];
Kp=4;Kd=.5*diag([4.12;12;12]);
t=ts*(0:N);
for i=1:N
    t(i)=i*ts;
    qdd(:,i)=zeros(3,1);
    % ***
    % trapezoidal trajectory
    % ***
    if t(i)<t_trap(end)
        qd(:,i)=interp1(t_trap,qdes',t(i))';        
    else
        qd(:,i)=qdes(:,end);
    end
    % PD control
    etaC(:,i)=-Kp*(xc(1:3,i)-qd(:,i))-Kd*(xc(4:6,i)-qdd(:,i));
    % sampled data system
    [tt,xx]=ode45(@(t,x) loaddyn(t,x,MIc,etaC(:,i)),[i-1 i]*ts,xc(:,i));
    % save next state
    xc(:,i+1)=xx(end,:)';
    maxlst = max(isnan(xx));
    maxnum = max(maxlst);
    if maxnum
        xc(:,i+1) = xc(:,i);
    end
end

figure(10);hold on;
roomshow(colobj,10);axis('square');
view(-90,90);axis([-1 11 -1 11 0 4]);
title('actual path')
robpath=xc([2;3;1],:);
for k=1:N
    robotshow(load,robpath(:,k));pause(.05);
end
hold off

figure(51);plot(t,xc(1,:),t(2:end),qd(1,:),t_trap,qdes(1,:),'LineWidth',2);
xlabel('time (s)');ylabel('angle (rad)');
legend('actual','desired','trapezoid');
title('orientation');
figure(52);plot(t,xc(2,:),t(2:end),qd(2,:),t_trap,qdes(2,:),'LineWidth',2);
xlabel('time (s)');ylabel('position (m)');
legend('actual','desired','trapezoid');
title('x position');
figure(53);plot(t,xc(3,:),t(2:end),qd(3,:),t_trap,qdes(3,:),'LineWidth',2);
xlabel('time (s)');ylabel('position (m)');
legend('actual','desired','trapezoid');
title('y position');


%
% load dynamics (for ODE solver)
%
function f=loaddyn(t,x,MIC,etaC)
    n2=size(x,1);
    n=fix(n2/2);
    f=[x(n+1:n2);inv(MIC)*etaC];
end
