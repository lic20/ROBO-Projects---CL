%
% landmark_est.m
%
% Given y and robot kinematics and assuming perfect localization (i.e., the
% robot state, estimate the landmark location pL and pB)
%
function [ahatnext,Pnext,Snext]=...
    landmark_est(ahat,q,u,y,N_range,N_bearing,N_odo,vcov,P,Sigma,robot,colobj)

yind=[(1:N_range) (N_range+N_odo+1:N_range+N_odo+N_bearing)];
N_a=3*length(yind);
A=eye(N_a,N_a);
V=diag(vcov(yind));

pLhat=reshape(ahat(1:3*N_range),3,N_range);
pBhat=reshape(ahat(3*N_range+1:3*N_range+3*N_bearing),3,N_bearing);

pC=zeros(N_range+N_bearing,N_a);
C(1:N_range,1:3*N_range)=gradient_range_pL(pLhat,q);
C(N_range+1:N_range+N_bearing,3*N_range+1:3*N_range+3*N_bearing)=...
    gradient_bearing_pB(pBhat,q);

K=P*C'*inv(V);

ybar=h(q,pLhat,pBhat);

ahatnext=ahat+K*(y(yind)-ybar);
Pnext=inv(inv(Sigma)+C'*inv(V)*C);
Snext=A*Pnext*A';

end

% range and bearing sensor output map

function y=h(q,pL,pB)
    pR=[q(1:2);0];
    y_range=vecnorm(pR-pL)';
    y_bearing=(atan2(pB(2,:)-q(2),pB(1,:)-q(1))-q(3))';
    y=[y_range;y_bearing];
end

% gradient of range sensor model

function grad_h=gradient_range_pL(pL,q)
    N_range=size(pL,2);
    pR=[q(1:2);0];
    grad_stack=((pL-pR)./vecnorm(pL-pR).^2)'; 
    grad_h=zeros(N_range,3*N_range);
    for i=1:N_range;grad_h(i,(i-1)*3+1:3*i)=grad_stack(i,:);end
end

% gradient of bearing sensor model

function grad_h=gradient_bearing_pB(pB,q)
    N_bearing=size(pB,2);
    pR=[q(1:2);0];
    grad_stack=[([-(pB(2,:)'-pR(2)) (pB(1,:)'-pR(1))]'./vecnorm(pB(1:2,:)-pR(1:2)).^2)' ...
        zeros(size(pB,2),1)];
    grad_h=zeros(N_bearing,3*N_bearing);
    for i=1:N_bearing;grad_h(i,(i-1)*3+1:3*i)=grad_stack(i,:);end
end