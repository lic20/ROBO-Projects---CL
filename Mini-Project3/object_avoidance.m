function new_q = object_avoidance(n,q,robot,colobj,qf)
    %potential field paramters:
    eta = 0.001;
    rho0 = 0.5;
    max_dq = [.1;.1]*5;
    alpha = 0.1;
    eps = 0.03;
    
    flatq = reshape(q,2,n);
    for i=1:length(flatq)
        [isInt,dist,wp]=colcheck(robot,[flatq(:,i);0],colobj);
        if max(isnan(dist)) > 0 || min(dist) < 0.1
            %disp('collision')
            U = U_attr(flatq(:,i),qf) + U_rep(flatq(:,i),robot,colobj,rho0,eta);
            grad_attr = gradU_attr(flatq(:,i),qf);
            grad_repel = gradU_rep(flatq(:,i),robot,colobj,rho0,eta);
            gradU = grad_attr + grad_repel;
            dq = -alpha*inv(eps*eye(2)+gradU'*gradU)*gradU'*U;
            
            deltaq = max(abs(dq)>max_dq)*dq/max(abs(dq./max_dq))+~max(abs(dq)>max_dq)*dq;
            flatq(:,i) = flatq(:,i) + deltaq;
            if norm(flatq(:,i)-qf) < rho0/2
                flatq(:,i) = flatq(:,i);
            else
                [isInt,dist,wp]=colcheck(robot,[flatq(:,i);0],colobj);
                while max(isnan(dist)) > 0
                    flatq(:,i) = flatq(:,i)+randn(2,1)*0.15;
                    [isInt,dist,wp]=colcheck(robot,flatq(:,i),colobj);
                end
            end
        end
    end
    new_q = reshape(flatq,n*2,1);
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