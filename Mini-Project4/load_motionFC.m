%
% 2D formation control example based on the relative pose in a room
% 
%
% contact force
%

% contact location
close all;
clear eta3_a; clear eta3_b; clear robots;
clear eta2_a; clear eta2_b;
clear G1_adj; clear G1; clear G3; clear G2;
clear eta_n_1a; clear eta_n_1b;

Pci_q=[10,80,190,260]*pi/180; %4 fingers
%Pci_q=[10,80,190,260 40]*pi/180; %5 fingers
%Pci_q=[10,80,190,260 40 120]*pi/180; %6 fingers
%Pci_q=[5,90,180,280]*pi/180; %4 fingers
%Pci_q=[5, 90, 270]*pi/180; % 3 fingers
%Pci_q=[5 180]*pi/180;
[Pci,e_ni]=contact_gen(Pci_q,load_x/2,load_y/2);


Pcorner=[load_x load_x -load_x -load_x load_x;...
    load_y -load_y -load_y load_y load_y]/2;
figure(60);plot(Pcorner(1,:),Pcorner(2,:),Pci(1,:),Pci(2,:),'x',...
    'LineWidth',2);
axis([-1,1,-1,1]);

% grasp map
K=[0 -1;1 0]; % planar cross product
m=length(Pci_q); % # of fingers

for k=1:N
    R=rot2(xc(1,k));
    Pci_0=R*Pci;
    %Pci_0_perp=K*Pci_0;
    Pci_0_perp=hat([0;0;1])*[Pci_0;zeros(1,m)];
    Pci_0_perp=Pci_0_perp(1:2,:);
    e_ni_0=R*e_ni;
    e_ti_0 = hat([0;0;1])*[e_ni_0;zeros(1,m)];
    e_ti_0 = e_ti_0(1:2,:);
    
    % rigid grasp
    G1{k}=zeros(3,2*m);
    for i=1:m
        G1{k}(1,(i-1)*2+1:2*i)=Pci_0_perp(:,i)';
        G1{k}(2:3,(i-1)*2+1:2*i)=eye(2);
        G1_adj{k}(:,(i-1)*3+1:3*i)=G1{k}(:,(i-1)*2+1:2*i)*...
            [e_ni_0(:,i) e_ti_0(:,i) -e_ti_0(:,i)];
    end
    mG1=size(G1_adj{k},2);
    
    [X_a,FVAL_a,EXITFLAG,OUTPUT] = quadprog(G1_adj{k}'*G1_adj{k}, zeros(mG1,1),-eye(mG1),-ones(mG1,1));
    [X_b,FVAL_b,EXITFLAG,OUTPUT] = quadprog(eye(mG1),zeros(mG1,1),-eye(mG1),-ones(mG1,1),G1_adj{k},zeros(3,1));
    if FVAL_a > 0.1
        disp('(a): No Force Closure Grasp For Rigid Contact')
        return
    end
    if isempty(X_b)
        disp('(b): No Force Closure Grasp For Rigid Contact')
        return
    end
    eta_1=pinv(G1_adj{k})*etaC(:,k);
    fun = @(x,eta_m,eta_s) -min(min(eta_m+eta_s*x),0); 
    x=(0:.1:10);
    for i=1:m
        i_1=(i-1)*3+1:3*i;
        x_a=fun(x,eta_1(i_1),X_a(i_1));
        x_b=fun(x,eta_1(i_1),X_b(i_1));
        ind=find(x_a>0);alpha_a(i)=x_a(min(ind));
        ind=find(x_b>0);alpha_b(i)=x_b(min(ind));
    end
    
    eta_a=pinv(G1_adj{k})*etaC(:,k)+max(alpha_a)*X_a;
    eta_b=pinv(G1_adj{k})*etaC(:,k)+max(alpha_b)*X_b;  
    
    for i=1:m
        eta_n_1a(i,k)=eta_a((i-1)*3+1);
        eta_n_1b(i,k)=eta_b((i-1)*3+1);
    end    
  
    % point contact without friction
    G2{k}=zeros(3,m);
    for i=1:m
        G2{k}(:,i)=G1{k}(:,(i-1)*2+1:2*i)*e_ni_0(:,i);
    end
    eta_2=pinv(G2{k})*etaC(:,k);
    mG2 = size(G2{k},2);
    [X_a,FVAL_a,EXITFLAG,OUTPUT] = quadprog(G2{k}'*G2{k}, zeros(mG2,1),-eye(mG2),-ones(mG2,1));
    [X_b,FVAL_b,EXITFLAG,OUTPUT] = quadprog(eye(mG2),zeros(mG2,1),-eye(mG2),-ones(mG2,1),G2{k},zeros(3,1));
    if FVAL_a > 0.1
        disp('(a): No Force Closure Grasp For Frictionless Contact')
        return
    end
    if isempty(X_b)
        disp('(b): No Force Closure Grasp For Frictionless Contact')
        return
    end
    x_a=fun(x,eta_2,X_a);
    x_b=fun(x,eta_2,X_b);
    ind=find(x_a>0);alpha_a=x_a(min(ind));
    ind=find(x_b>0);alpha_b=x_b(min(ind));
    
    eta2_a(:,k)=pinv(G2{k})*etaC(:,k)+max(alpha_a)*X_a;
    eta2_b(:,k)=pinv(G2{k})*etaC(:,k)+max(alpha_b)*X_b;    
    
    
    % point contact with friction
    phi=pi/4*ones(1,m); % friction cone
    G3{k}=zeros(3,2*m);
    for i=1:m
        G3{k}(:,(i-1)*2+1:2*i)=G1{k}(:,(i-1)*2+1:2*i)*...
            [rot2(phi(i))*e_ni_0(:,i) rot2(-phi(i))*e_ni_0(:,i)];
    end  
    eta_3=pinv(G3{k})*etaC(:,k);
    mG3 = size(G3{k},2);
    [X_a,FVAL_a,EXITFLAG,OUTPUT] = quadprog(G3{k}'*G3{k}, zeros(mG3,1),-eye(mG3),-ones(mG3,1));
    [X_b,FVAL_b,EXITFLAG,OUTPUT] = quadprog(eye(mG3),zeros(mG3,1),-eye(mG3),-ones(mG3,1),G3{k},zeros(3,1));
    if FVAL_a > 0.1
        disp('(a): No Force Closure Grasp For Friction Contact')
        return
    end
    if isempty(X_b)
        disp('(b): No Force Closure Grasp For Friction Contact')
        return
    end
    x_a=fun(x,eta_3,X_a);
    x_b=fun(x,eta_3,X_b);
    ind=find(x_a>0);alpha_a=x_a(min(ind));
    ind=find(x_b>0);alpha_b=x_b(min(ind));
    
    eta3_a(:,k)=pinv(G3{k})*etaC(:,k)+max(alpha_a)*X_a;
    eta3_b(:,k)=pinv(G3{k})*etaC(:,k)+max(alpha_b)*X_b;   
    
    
    %compute robot location:
    loc = xc(2:3,k) + rot2(xc(1,k))*Pci;
    robots(:,k) = reshape(loc,m*2,1);
end

figure(21);plot(t(1:end-1),eta_n_1b,'linewidth',2);legend('1','2','3','4');
title('rigid grasp normal force')
figure(22);plot(t(1:end-1),eta2_b(1:m,:),'linewidth',2);legend('1','2','3','4');
title('frictionless grasp normal force')
figure(23);plot(t(1:end-1),eta3_b(1:m,:),'linewidth',2);legend('1','2','3','4');
title('frictional grasp force')

%show the load and the robots
figure(5);
roomshow(colobj,5)
for k=1:N
    if mod(k,5) == 0
        robot_loc = reshape(robots(:,k),2,m);
        load.Pose(1:3,4)=[robpath(1,k);robpath(2,k);0];
        load.Pose(1:3,1:3)=rot([0;0;1],robpath(3,k));
        [~,patchObj] = show(load);
        patchObj.FaceColor = [218,165,32]/218;
        patchObj.FaceColor = [1 1 1];
        plot(robot_loc(1,:),robot_loc(2,:),'x','LineWidth',2)
    end
end

%
% grasping robots location
% 

%
% generate contacts based on load dimension and angle
%
function [P,Pn]=contact_gen(q,b,a)

P=zeros(2,length(q));
Pn=P;
for i=1:length(q)
    if abs(b*tan(q(i)))<a
        if(abs(q(i))<pi/2)
            P(2,i)=b*tan(q(i));
            P(1,i)=b;
            Pn(:,i)=[-1;0];
        else
            P(2,i)=-b*tan(q(i));
            P(1,i)=-b;
            Pn(:,i)=[1;0];
        end
    else
        if (q(i)<pi)&&(q(i)>0)
            P(2,i)=a;
            P(1,i)=a*tan(pi/2-q(i));
            Pn(:,i)=[0;-1];
        else
            P(2,i)=-a;
            P(1,i)=-a*tan(pi/2-q(i));
            Pn(:,i)=[0;1];
        end        
    end
end
end

%
% planar rotation
%

function R=rot2(q)

R=[cos(q) -sin(q); sin(q) cos(q)];

end 
