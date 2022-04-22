%
% predictive control for nonholonomic systems with obstacle avoidance
%

clear all;close all;

% define room
colobj=roomspec;

% define robot
robot=collisionBox(1,0.5,1);

% initial condition
q0=[1;1;pi/2];

qf = [6.27;7.98;0]; % Kp=.5; dmin=.2; N=20; k_pot=.06
fprintf('Target goal is [%.2f %.2f]\n',qf(1),qf(2))

% trajopt planner (planar translation)
n=3; % # of states
x0=q0(1:n);
xf=qf(1:n);

% control barrier function (sigma function)
dmin=.2; % collision safety distance 
sigmafun=@(hI,eta,c,m,e) (hI>eta).*(-m*atan(c*(hI-eta))*2/pi)+...
    (hI>0).*(hI<eta).*(e*(eta-hI)/eta)+(hI<=0).*e;
eta=.2;c=100;M=50;repel=10;

% parameters
N=20; % prediction horizon
Nmax=100; % total horizone
ts=.1; % sampling period
m=2;n=3;% # of input, # of state

%
k_pot=.06;

% constraints
umx=[4;2];
ubarmx=reshape(umx*ones(1,N),m*N,1);
x3min=-pi;
x3max=pi;

% initial guess
u0=.1*rand(m,N);
%u0=.5*[1;0]*ones(1,N);
u0bar=reshape(u0,N*m,1);

x=zeros(n,Nmax+1);
u=zeros(m,Nmax);
x(:,1)=x0;
xbar=zeros(n*N,N);
ubar=zeros(m*N,N);
ubar(:,1)=u0bar;
xbar(:,1)=traj(x0,u0bar,m,ts);

epsilon=.8;
Kp=diag([1;1;.1])*.7;

% show room
roomshow(colobj,12);
view(-90,90);axis([-1 11 -1 11 0 1]);grid;axis('square')

% main control loop 

for i=1:Nmax
    J=trajgrad(xbar(:,i),ubar(:,i),n,m,ts);
    x_NstepAhead=xbar((N-1)*n+1:N*n,i);

% Formulate the constraints
    % 
    % check collision of x_k
    % 
    Aineq=zeros(N-1,m*N);bineq=zeros(N-1,1);
    Ax3ineqmx=zeros(N-1,m*N);bx3ineqmx=zeros(N-1,1);
    Ax3ineqmn=zeros(N-1,m*N);bx3ineqmn=zeros(N-1,1);
    for k=2:N
        x_subk = xbar((k-1)*n+1:k*n,i);
        [isInt,dist,wp]=colcheck(robot,x_subk,colobj);
        [mindist(k),ind]=min(dist); 
        d=(wp{ind}(1:2,1)-wp{ind}(1:2,2));
        grad_d_x = grad_numerical(d,x_subk,robot,colobj);
        J_subk = trajgrad_k(k,xbar(:,i),ubar(:,i),n,m,ts);
        if norm(d)<dmin
            %disp([i,k,norm(d)]);
            Aineq(k-1,:)=-d'*grad_d_x*J_subk;
            %Bineq(k-1)=-(sigmafun(.5*d'*d,a,b,e,l));
            Bineq(k-1)=sigmafun(.5*d'*d,eta,c,M,repel);
        end
        if x_subk(3)<x3min+.05
            Ax3ineqmn=-J_subk(3,:);
            %bx3ineqmn=-sigmafun(x_subk(3)-x3min,a,b,e,l);
            bx3ineqmn=-sigmafun(x_subk(3)-x3min,eta,c,M,repel);
        end
        if x_subk(3)>x3max-.05
            Ax3ineqmx=J_subk(3,:);
            %bx3ineqmx=-sigmafun(x3max-x_subk(3),a,b,e,l);
            bx3ineqmx=-sigmafun(x3max-x_subk(3),eta,c,M,repel);
        end
    end

    A=[Aineq;Ax3ineqmn;Ax3ineqmx];
    B=[bineq;bx3ineqmn;bx3ineqmx];
    % solve quadratic programming problem
    H = J'*J + epsilon*eye(m*N);
    F = J'*(Kp*(x_NstepAhead-xf));
    opt = optimset('display','off');
    
    delta_ubar = quadprog(H,F,A,B,[],[],...
            -ubarmx-ubar(:,i),ubarmx-ubar(:,i),zeros(m*N,1),opt);

    ubar_next=ubar(:,i) + delta_ubar;

    pred_collision = 1;

    %
    alpha=1;
    pot_gain=1;
    E=epsilon*diag(reshape(ones(m,1)*(1:.1:1+(N-1)*.1),m*N,1));
    %E=epsilon*eye(m*N);
    ubar_pot=zeros(m*N,1);
    while pred_collision > 0
        pred_collision = 0;
        %u(:,i)=ubar((N-1)*m+1:N*m,i+1);
        u(:,i)=ubar_next(1:m);
        x(:,i+1)=fu_disc(x(:,i),u(:,i),ts);
        ubar(:,i+1)=[ubar_next(m+1:m*N);zeros(m,1)];
        xbar(:,i+1)=traj(x(:,i+1),ubar(:,i+1),m,ts);
        %
        for k=2:N
            x_subk = xbar((k-1)*n+1:k*n,i+1);
            [isInt,dist,wp]=colcheck(robot,x_subk,colobj);
            if (max(isnan(dist))>0)||min(dist)<dmin/1.2
                pred_collision=1;
                J_k = trajgrad_k(k,xbar(:,i),ubar(:,i),n,m,ts);
                ubar_pot=ubar_pot+...
                    (gradU1_rep(x_subk,robot,colobj,2*dmin,k_pot)*J_k)';
%                k_hit=k;
            end
        end
        % 
        if pred_collision>0
            alpha=alpha/1.5;
            F = alpha*J'*(Kp*(x_NstepAhead-xf));            
            delta_ubar = quadprog(H,F,A,B,[],[],...
                -ubarmx-ubar(:,i),ubarmx-ubar(:,i),zeros(m*N,1),opt);
            if alpha < .1
                pot_gain = pot_gain*1.1;
                delta_ubar= delta_ubar + pot_gain*ubar_pot;                    
            end
            ubar_next=ubar(:,i) + delta_ubar;
        end
    end
    %
    figure(12);robotshow(robot,x(:,i+1));   
end


disp([x(:,Nmax+1) xf]);

t=ts*(1:Nmax+1);

figure(4);plot(t,x,t,xf*ones(1,Nmax+1),':','linewidth',2);
legend('x','y','\theta');
title('vehicle state vs. time');
figure(2);plot(x(1,:),x(2,:),xf(1),xf(2),'x','linewidth',2);
xlabel('x');ylabel('y');grid;axis('square')
title('vehicle motion in work space')
view(-90,90);axis([-1 11 -1 11]);
figure(3);plot(t(1:end-1),u,'linewidth',2)
legend('v','\omega');
title('vehicle input vs. time');

despath = x;
save despath.mat despath

% ******************************%
% functions used in the program %
% ******************************%

function xbar=traj(x0,ubar,m,ts)
    n=length(x0);
    Nm=length(ubar);N=Nm/m;
    u=reshape(ubar,m,N);
    x=zeros(n,N+1);
    x(:,1)=x0;
    for i=1:N
        x(:,i+1)=fu_disc(x(:,i),u(:,i),ts);
    end
    xbar=reshape(x(:,2:N+1),n*N,1);
end

function J=trajgrad(xbar,ubar,n,m,ts)
    Nm=length(ubar);N=Nm/m;
    u=reshape(ubar,m,N);
    x=reshape(xbar,n,N);
    J=zeros(n,N*m);
    for i=1:N
        [A,B]=grad_fu(x(:,i),u(:,i),ts);
        J=A*J;
        J(:,(i-1)*m+1:m*i)=B;
    end
end

%
% compute the Jacobian for x_k
%
function J_k=trajgrad_k(k,xbar,ubar,n,m,ts)
    Nm=length(ubar);N=Nm/m;
    u=reshape(ubar,m,N);
    x=reshape(xbar,n,N);
    J_k=zeros(n,N*m);
    for i=1:k
        [A,B]=grad_fu(x(:,i),u(:,i),ts);
        J_k=A*J_k;
        J_k(:,(i-1)*m+1:m*i)=B;
    end
end

function x_new=fu_disc(x,u,ts)
    n=length(x);
    x_new = (x+ts*fu_func(x,u));
end

% nonholonomic system continuous vector field
function fu=fu_func(x,u)
    fu=[cos(x(3)) 0;sin(x(3)) 0;0 1]*u;
end

function f=f_func(x)
    f=[cos(x(3)) 0;sin(x(3)) 0;0 1];
end
% gradient system about a given trajectory
function [A,B]=grad_fu(x,u,ts)
    n=length(x);
    A=eye(n)+ts*[0 0 -u(1)*sin(x(3));0 0 u(1)*cos(x(3));0 0 0];
    B=ts*f_func(x);
end

% numerical gradient

function grad_d_x = grad_numerical(d,x,robot,colobj)

    nd=length(d);
    n=length(x);
    epsilon=.01;
    grad_d_x=zeros(nd,n);
    for i=1:n
        e_i=zeros(n,1);e_i(i)=1;
        [isInt,dist,wp]=colcheck(robot,x+epsilon*e_i,colobj);
        [mindist,ind]=min(dist);    
        d_epsilon=(wp{ind}(1:2,1)-wp{ind}(1:2,2));
        grad_d_x(:,i)=(d_epsilon-d)/epsilon;
    end
end

% rotation matrix

function R=rot(k,theta)
  
  k=k/norm(k);
  R=eye(3,3)+sin(theta)*hat(k)+(1-cos(theta))*hat(k)*hat(k);
  
end

% cross product matrix 
function khat = hat(k)
  
  khat=[0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];

end

% potential function for ubar

function gradx=gradU1_rep(q,robot,colobj,rho0,eta)

    e=.01;
    [isInt,dist,wp]=colcheck(robot,q,colobj);   
    [rho,ind]=min(dist);

    [isInt,dist1,wp]=colcheck(robot,q+e*[1;0;0],colobj);   
    [rho1,ind]=min(dist1);

    [isInt,dist2,wp]=colcheck(robot,q+e*[0;1;0],colobj);   
    [rho2,ind]=min(dist2);

    [isInt,dist3,wp]=colcheck(robot,q+e*[0;0;1],colobj);   
    [rho3,ind]=min(dist3);

    gradrho=[(rho1-rho)/e (rho2-rho)/e (rho3-rho)/e];

    gradx=[0 0 0 ];
    if rho<rho0
        gradx=eta*(rho-rho0)*gradrho;
    end

end