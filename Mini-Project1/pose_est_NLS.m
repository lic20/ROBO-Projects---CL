%
% pose_est_NLS.m
%
% Given y, use nonlinear least square method to estimate the robot position
% and orientation 
%
function [qhat,delta_y]=pose_est_NLS(qhat0,y,pL,pB)

% use range sensors first
N_range=size(pL,2);N_bearing=size(pB,2);N_odo=2;

qhat(:,1)=qhat0.*(1+.01*randn(3,1));

% iteration
alpha=.3; % step size
epsilon=.1; % LM parameter
N=15; % # of iterations

m=N_range+N_bearing;

for i=1:N
    yhat_RB=h(qhat(:,i),pL,pB);
    J=[gradient_range(pL,qhat(:,i));gradient_bearing(pB,qhat(:,i))];
    %J=stoch_grad(qhat(:,i),@h,pL,pB);
    qhat(:,i+1)=qhat(:,i)-...
        alpha*inv(epsilon*eye(3,3)+J'*J)*J'*(yhat_RB-[y(1:4);y(7:8)]);
    delta_y(:,i)=yhat_RB-[y(1:4);y(7:8)];
    if (i>1)&(abs(norm(delta_y(:,i))-norm(delta_y(:,i-1)))<1e-4)
        break
    end
end

end

function y=h(q,pL,pB)
    pR=[q(1:2);0];
    y_range=vecnorm(pR-pL)';
    y_bearing=(atan2(pB(2,:)-q(2),pB(1,:)-q(1))-q(3))';
    y=[y_range;y_bearing];
end

function J=stoch_grad(x,fun,pL,pB)

n=length(x);
N=10*n;

e=norm(x)*1e-6;
y0=feval(fun,x,pL,pB);
dx=e*randn(length(x),N);

for i=1:10*n
    y(:,i)=feval(fun,x+dx(:,i),pL,pB);
end

J=(y-y0)*pinv(dx);

end

function grad_h=gradient_range(pL,q)
    pR=[q(1:2);0];
    grad_h=(-(pL-pR)./vecnorm(pL-pR))';    
end

function grad_h=gradient_bearing(pB,q)
    pR=[q(1:2);0];
    grad_h=[([pB(2,:)'-pR(2) -(pB(1,:)'-pR(1))]'./vecnorm(pB(1:2,:)-pR(1:2)).^2)' ...
        -ones(size(pB,2),1)];
end