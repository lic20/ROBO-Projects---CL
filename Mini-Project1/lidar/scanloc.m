%
% scanloc.m
%
% find the end point of scan line using current robot configureation
%

function [xloc,yloc]=scanloc(y,q)

M=size(y,1); % # of scan lines at each time
phi=2*pi*(0:M-1)/M;
xloc=zeros(size(y));
yloc=zeros(size(y));
for i=1:size(y,2)
    for j=1:M        
        loc=q(1:2,i)+ [cos(q(3,i)+phi(j));sin(q(3,i)+phi(j))]*y(j,i);
        xloc(j,i)=loc(1);
        yloc(j,i)=loc(2);
    end    
end

end