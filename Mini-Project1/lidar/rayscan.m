function lmax=rayscan(robot,q,colobj,phi)

rz=robot.Z;
ex=[1;0;0];ey=[0;1;0];ez=[0;0;1];zv=[0;0;0];
l0=2;r=.01;
lmax=bisectsearch(l0,colobj,r,q,rz,phi);

end

function l=bisectsearch(l0,colobj,r,q,rz,phi)

    l1=0;l2=l0;lmin=.1;
       
%   if l2 is feasible, then let l1=l2 and increase l2
%   if l2 is infeasible, then lower l2 by (l2-l1)/2
    imax=10;i=1;
    while (i<imax)&&((l2-l1)>lmin)
        ray=defineray(l2,r,q,rz,phi);
%        ray.Length=l2;
        for j=1:length(colobj.obj)
            [isInt(j),dist(j),wp{j}]=checkCollision(ray,colobj.obj{j});
        end
        if max(isnan(dist))>0;
            l2=l2-(l2-l1)/2;
        else
            l=l2+(l2-l1);l1=l2;l2=l;
        end
        i=i+1;
    end
    l=(l2-l1)/2+l1;
end

