%
% wmr.m
%
% [qnext,utrue]=wmr(q,u,ts,wcov): discrete time evolution of a wheeled mobile robot
% 
% q=current state
% u=current input
% ts=sampling period (s)
% Gaussian odometry noise (zero mean)
%
% qnext=next state
% utrue=true input (with noise and disturbance)
%

function [qnext,utrue]=wmr(q,u,ts,wcov)

m=length(u);
w=zeros(size(u));
w=normrnd(w,wcov(1:m));
utrue=u+w;
qnext(3)=q(3)+utrue(2)*ts;
qnext(1)=q(1)+utrue(1)*cos(qnext(3))*ts;
qnext(2)=q(2)+utrue(1)*sin(qnext(3))*ts;

end
