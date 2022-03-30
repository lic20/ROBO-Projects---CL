%This is the function for Kalman filter estimation

function [qhat_next, P_next, S_next] = ...
    pose_est_filter(qhat,u,y,pL,pB,vcov,P,Sigma)

wcov = 0.05*ones(2,1);
sigma=noise(wcov);
qbar_next = qhat + u + sigma;

A = eye(2);
B = eye(2);
V = diag(vcov);
W = diag(wcov);

C=[gradient_range(pL,qbar_next);gradient_bearing(pB,qbar_next)];

K = P*C'*inv(V);
ybar = h(qbar_next,pL,pB);
qhat_next = qbar_next + K*(y-ybar);
P_next = inv(inv(Sigma)+C'*inv(V)*C);
S_next = A*P_next*A' + B*W*B';
end

function w = noise(wcov)
    w = zeros(2,1);
    w = normrnd(w,wcov(1:2));
end

function y=h(q,pL,pB)
    pR=[q(1:2);0];
    y_range=vecnorm(pR-pL)';
    y_bearing=(atan2(pB(2,:)-q(2),pB(1,:)-q(1)))';
    y=[y_range;y_bearing];
end

% gradient of range sensor model

function grad_h=gradient_range(pL,q)
    pR=[q(1:2);0];
    grad_h=(-(pL-pR)./vecnorm(pL-pR).^2)';    
    grad_h = grad_h(:,1:2);
end

% gradient of bearing sensor model

function grad_h=gradient_bearing(pB,q)
    pR=[q(1:2);0];
    grad_h=[([pB(2,:)'-pR(2) -(pB(1,:)'-pR(1))]'./vecnorm(pB(1:2,:)-pR(1:2)).^2)'];
end
