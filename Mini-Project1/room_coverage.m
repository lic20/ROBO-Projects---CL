%Room coverage algorithm

clear all;close all
rng('shuffle')

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
robotshow(robot,q0);

%initialize u(t)
a = 0; b = pi;
theta = a + (b-a).*rand(1,1);
q0(3) = theta;
disp(q0)
v = 0.1;
w = 0.1;
current_u = [v,0];
steps = 20;
current_q=wmr_true(q0,current_u);
collision = check_collision(robot,current_q,colobj);

for k=1:steps
    if collision == 1
        current_u(1) = 0.1;
        current_u(2) = 0;
        temp_q = current_q;
        while collision
            theta = (2*pi).*rand(1,1);
            temp_q(3) = theta;
            temp_q=wmr_true(temp_q,current_u);
            collision = check_collision(robot,temp_q,colobj);
        end
        current_q(3) = temp_q(3);
        disp(current_q)
        continue
    end
    robotshow(robot,current_q);
    while collision == 0
        u = [v;0];
        current_q=wmr_true(current_q,u);
        robotshow(robot,current_q);
        collision = check_collision(robot,current_q,colobj);
    end
end

function col_true = check_collision(robot,q,colobj)
    col_true = 0;
    [isInt,dist,wp]=colcheck(robot,q,colobj);   
    if (max(isnan(dist))>0) | min(dist) < 0.05
        col_true = 1;
    end
end

%wmr without noise
function qnext = wmr_true(q,u)
    qnext = zeros(3,1);
    qnext(3)=q(3)+u(2);
    qnext(1)=q(1)+u(1)*cos(qnext(3));
    qnext(2)=q(2)+u(1)*sin(qnext(3));
end

