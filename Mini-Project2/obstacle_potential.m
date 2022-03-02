%Moving obstacle with potential method
%Note that the collision object of the chair is stored in number 9 in
%colobj
clear all;close all
colobj = roomspec();
roomshow(colobj);
axis('square');
view(-90,90);

rL=.4;rW=.2;rz=.1;
robot=collisionCylinder(rW,rz);

q0 = [.4;.2];
robot_show(robot,q0);

%get target position
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
qf = qf(1:2);
fprintf('Target goal is [%.2f %.2f]\n',qf(1),qf(2))

%potential field parameters
eta = 0.001;
rho0 = 0.5;
max_dq = [.1;.1]*5;
alpha = 0.1;
eps = 0.03;

q(:,1) = q0;
chair_pos(:,1) = [2;7];

steps = 200;
for k=1:steps
    chair_pos(:,k) = colobj.obj{9}.Pose(1:2,4);
    M(k) = getframe;
     %LM method
    U = U_attr(q(:,k),qf) + U_rep(q(:,k),robot,colobj,rho0,eta);
    grad_attr = gradU_attr(q(:,k),qf);
    grad_repel = gradU_rep(q(:,k),robot,colobj,rho0,eta);
    gradU = grad_attr + grad_repel;
    dq = -alpha*inv(eps*eye(2)+gradU'*gradU)*gradU'*U;
    
    deltaq = max(abs(dq)>max_dq)*dq/max(abs(dq./max_dq))+~max(abs(dq)>max_dq)*dq;
    
    q(:,k+1) = q(:,k)+deltaq;
    if norm(q(:,k)-qf) < rho0/2
        q(:,k+1) = q(:,k+1);
    else
        [isInt,dist,wp]=colcheck(robot,q(:,k+1),colobj);
        while max(isnan(dist)) > 0
            q(:,k+1) = q(:,k)+randn(2,1)*0.3;
            [isInt,dist,wp]=colcheck(robot,q(:,k+1),colobj);
            if max(isnan(dist)) == 0
                break;
            end
        end
    end
    robot_show(robot,q(:,k+1))
    
    t = 0.01*k;
    colobj = move_obstacle(t,colobj,[2;7]);
end

movie(M)

%==================================
%Functions
%==================================

function new_colobj = move_obstacle(time,colobj,pose_original)
    A = 6; T = 0.3;
    
    new_pose = pose_original - A*sin(2*pi*time/T)*[0;1];
    if new_pose(2) >= 9.75
        new_pose(2) = 9.75;
    elseif new_pose(2) <= 0.25
        new_pose(2) = 0.25;
    end
    colobj.obj{9}.Pose(1:2,4) = new_pose;
    [~,patchObj] = show(colobj.obj{9});    
    patchObj.FaceColor = colobj.color{9};
    patchObj.EdgeColor = 'none';
    new_colobj = colobj;
end

function robot_show(robot,q)
    robot.Pose(1:3,4) = [q;0];
    show(robot);
end

%potential functions
function U = U_attr(q,qf)
    U = 0.5*norm(q-qf)^2;
end

function gradU = gradU_attr(q,qf)
    gradU = (q-qf)';
end

function U = U_rep(q,robot,colobj,rho0,eta)
    [isInt,dist,wp]=colcheck(robot,q,colobj);
    U = 0;
    for i=1:length(dist)
        rho = dist(i);
        if isnan(rho)
            continue
        end
        U = U + (rho<rho0)*(eta/2)*(1/rho-1/rho0)^2;
    end
end

function gradU = gradU_rep(q,robot,colobj,rho0, eta)
    [isInt,dist,wp]=colcheck(robot,q,colobj);
    gradU = [0 0];
    for k=1:length(dist)
        rho = dist(k);
        if isnan(rho) || rho >= rho0
            continue
        end
        iteration = 12;
        e = 0.01;
        for i=1:iteration
            dq_i = (rand(2,1)-0.5)*2*e;
            current_q = q+dq_i;
            B(:,i) = dq_i;
            [isInt,dist,wp]=colcheck(robot,current_q,colobj);
            A(:,i) = dist(k) - rho;
        end
        gradrho = A*pinv(B);
        if max(isnan(gradrho)) > 0
            continue
        end
        gradU_i = (-eta/rho^2)*(1/rho-1/rho0)*gradrho;
        gradU = gradU + gradU_i;
    end
end