%
% example program of showing the scan pattern at a random point on the
% robot path -- assume a state trajectory already exists in q (3xN)
% 
% calls scanpattern.m to generate the scan lines
% calls roomshow and robotshow to visualize locaiton and obstacles
%
% M = # of scan lines
% 
% output: 
% l = vector of distance to obstacles (first element = heading
% direction then counterclockwise)
%
fignum=3;
% check if figure exists
if ishandle(fignum);close(fignum);end
% show room in figure 3
roomshow(colobj,fignum);axis('square');
% choose a robot configuration
tmax=size(q,2);
ii=fix(rand*tmax);
qq=q(:,ii);
% show robot
robotshow(robot,qq);
% show the scan lines
l=scanpattern(qq,robot,colobj,N_scan);

