%This is the function for Kalman filter estimation

function [qhat_next, P_next, S_next] = ...
    pose_est_filter(qhat,u,y,ts,pL,pB,N_scan,wcov,vcov,P,Sigma, robot,colobj)

N_range=size(pL,2);N_bearing=size(pB,2);N_odo=2;
[qbar_next, utrue] = wmr(qhat,u,ts,(zeros(size(wcov))));
qbar_next = qbar_next';
yind=[(1:N_range) (N_range+N_odo+1:N_range+N_odo+N_bearing)];

A = [1 0 -u(1)*sin(qhat(3));0 1 u(1)*cos(qhat(3)); 0 0 1];
B = [cos(qhat(3)) 0; sin(qhat(3)) 0; 0 1];
V = diag(vcov(yind));
W = diag(wcov);

C=[gradient_range(pL,qbar_next);gradient_bearing(pB,qbar_next)];

K = P*C'*inv(V);
ybar = h(qbar_next,pL,pB);
qhat_next = qbar_next + K*(y(yind)-ybar);
P_next = inv(inv(Sigma)+C'*inv(V)*C);
S_next = A*P_next*A' + B*W*B';
end

function y=h(q,pL,pB)
    pR=[q(1:2);0];
    y_range=vecnorm(pR-pL)';
    y_bearing=(atan2(pB(2,:)-q(2),pB(1,:)-q(1))-q(3))';
    y=[y_range;y_bearing];
end

% gradient of range sensor model

function grad_h=gradient_range(pL,q)
    pR=[q(1:2);0];
    grad_h=(-(pL-pR)./vecnorm(pL-pR).^2)';    
end

% gradient of bearing sensor model

function grad_h=gradient_bearing(pB,q)
    pR=[q(1:2);0];
    grad_h=[([pB(2,:)'-pR(2) -(pB(1,:)'-pR(1))]'./vecnorm(pB(1:2,:)-pR(1:2)).^2)' ...
        -ones(size(pB,2),1)];
end
