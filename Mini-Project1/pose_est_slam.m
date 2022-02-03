%Localization and Mapping for SLAM
function [xhat_next, Pnext, Snext] = ...
    pose_est_slam(xhat,u,y,ts,N_range,N_bearing,N_odo,wcov,vcov,P,Sigma, robot,colobj)
 
yind=[(1:N_range) (N_range+N_odo+1:N_range+N_odo+N_bearing)];
N_a=21;

pLhat=reshape(xhat(4:15),3,N_range);
pBhat=reshape(xhat(16:21),3,N_bearing);
qhat = xhat(1:3);

A=eye(N_a,N_a);
A(1:3,1:3) = [1 0 -u(1)*sin(qhat(3));0 1 u(1)*cos(qhat(3)); 0 0 1];
V=diag(vcov(yind));

%pC=zeros(N_range+N_bearing,N_a);
C(1:4,:)= gradient_range_pL(pLhat,qhat);
C(5:6,:)= gradient_bearing_pB(pBhat,qhat);

K=P*C'*inv(V);
ybar=h(qhat,pLhat,pBhat);
xhat_next=xhat+K*(y(yind)-ybar);
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

% gradient of range sensor model,including robot position q

function grad_h=gradient_range_pL(pL,q)
    N_range=size(pL,2);
    pR=[q(1:2);0];
    grad_stack2=((pL-pR)./vecnorm(pL-pR).^2)';  %for sensor position
    temp_pL = pL; temp_pL(3,:) = zeros(1,4);
    grad_stack1=(-(temp_pL-pR)./vecnorm(pL-pR).^2)'; %for robot q
    grad_h=zeros(N_range,21);
    for i=1:N_range;grad_h(i,1:3)=grad_stack1(i,:);end
    for i=1:N_range;grad_h(i,3*i+1:3*i+3)=grad_stack2(i,:);end
end

% gradient of bearing sensor model,including robot position q

function grad_h=gradient_bearing_pB(pB,q)
    N_bearing=size(pB,2);
    pR=[q(1:2);0];
    grad_stack1=[([pB(2,:)'-pR(2) -(pB(1,:)'-pR(1))]'./vecnorm(pB(1:2,:)-pR(1:2)).^2)' ...
        -ones(size(pB,2),1)]; %robot q
    grad_stack2=[([-(pB(2,:)'-pR(2)) (pB(1,:)'-pR(1))]'./vecnorm(pB(1:2,:)-pR(1:2)).^2)' ...
        zeros(size(pB,2),1)]; %bearing position
    grad_h=zeros(N_bearing,21);
    for i=1:N_bearing;grad_h(i,1:3)=grad_stack1(i,:);end
    grad_h(1,16:18) = grad_stack2(1,:);
    grad_h(2,19:21) = grad_stack2(2,:);
end