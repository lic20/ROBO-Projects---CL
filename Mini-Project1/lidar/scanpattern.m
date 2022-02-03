%
% l=scanpattern(q,robot,colobj,M)
%
% calls defineray and rayscan
%
% q=robot state
% robot=robot collision object
% colobj= room collision objects
% M= # of scan lines
%
% l=array of M scan distances
%
function l=scanpattern(q,robot,colobj,M,nofig)

phi=2*pi*[0:M-1]/M;

r=.01;
rz=robot.Z;

for i=1:M;
    l(i)=rayscan(robot,q,colobj,phi(i));
    ray=defineray(l(i),r,q,rz,phi(i));
    if ~exist('nofig')
        show(ray);
    end
end

end
