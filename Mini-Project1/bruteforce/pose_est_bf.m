%
% pose_est.m
%
% Given y and robot kinematics, estimate the robot position and orientation
%
function qhat=pose_est_bf(y,pL,pB,N_scan,wcov,vcov)

% use range sensors first
N_range=size(pL,2);N_bearing=size(pB,2);N_odo=2;

% use all permutations of 3 balls out of all range measurements

ind=nchoosek(1:N_range,3);

for i=1:size(ind,1)
    r_adj=1;dr=.01;
    z(:,2*i-1:2*i)=threeballs(pL(:,ind(i,1)),pL(:,ind(i,2)),pL(:,ind(i,3)),...
        y(ind(i,1)),y(ind(i,2)),y(ind(i,3)));
    while(isnan(z(:,2*i-1:2*i)))&(r_adj<1.5)
        r_adj=r_adj+dr;
        z(:,2*i-1:2*i)=threeballs(pL(:,ind(i,1)),pL(:,ind(i,2)),pL(:,ind(i,3)),...
        y(ind(i,1))*r_adj,y(ind(i,2))*r_adj,y(ind(i,3))*r_adj);
    end
end

indnan=find(isnan(z));z(indnan)=[];z=reshape(z,3,length(z)/3);

if isempty(z)
    pRest=[];
else
    pRest=mean(z(1:2,:)')';
end

% use two range sensors intersecting with the x-y plane
ez=[0;0;1];
ind=nchoosek(1:N_range,2);
for i=1:size(ind,1)
    zz=two_range_sensor_and_plane(pL(:,ind(i,1)),pL(:,ind(i,2)),...
       y(ind(i,1)),y(ind(i,2)),ez,zeros(3,1));
    j = 1;
    if ~isempty(pRest)
        [minval,j]=min(vecnorm(zz(1:2,1:2)-pRest(:,1)));
    else
        if zz(2,2) > zz(2,1)
            j = 2;
        end
    end
    z1(:,i)=zz(:,j);
end
indnan=find(isnan(z1));z1(indnan)=[];z1=reshape(z1,3,length(z1)/3);
if ~isempty(z1)
    pRest=mean(z1(1:2,:)')';
end

% use two bearing sensors and a range sensor

for i=1:N_range 
    [zz,qq]=two_bearing_one_range(pB,pL(:,i),...
        y(N_range+N_odo+1:N_range+N_odo+2),y(i));
    [minval,j]=min(vecnorm(zz(1:2,1:2)-pRest(:,1)));
    z2(:,i)=zz(:,j);q2(:,i)=qq(j);
end
indnan=find(isnan(z2));z2(indnan)=[];z2=reshape(z2,3,length(z2)/3);
indnan=find(isnan(q2));q2(indnan)=[];
qRest3=mean(q2);
if size(z2,2)>1
  pRest3=mean(z2(1:2,:)')';
else
  pRest3=z2(1:2,:);
end


% only use 2 range sensors intersecting plane method
for i=1:N_bearing
    qRest(i)=subprob1(ez,[1;0;0],...
        (pB(:,i)-[pRest;0])/norm(pB(:,i)-[pRest;0]))-y(N_range+N_odo+i);
end
qhat=[pRest;mean(qRest)];

end