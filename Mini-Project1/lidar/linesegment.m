%
% line segment.m
%
% identify line segment from planar point dataset
%

function [ept,segmentlength,indrange,unitvec,mb,x,y]=...
    linesegment(pt,thresh,numseg,minlength,fignum);

%thresh=.1;
%pt=[xloc1 yloc1];fignum=80;
%pt=[xloc2 yloc2];fignum=81;
% form vectors between consecutive points
dpt=diff(pt);
% normalize these vector
ept=dpt./vecnorm(dpt')';
% shift these vectors by one
eptshift=ept(2:length(ept),:);
% dot product of consecutive unit vector, close to 1 means nearly aligned
eptdot=abs(dot(ept(1:length(ept)-1,:)',eptshift')-1);
% find those nearly aligned
ind=find(eptdot<thresh);
% find the length of the segments
ind1=find(diff(diff(ind)==1)==1)+2;
ind2=find(diff(diff(ind)==1)==-1)+2;
if(length(ind2)<length(ind1));
    ind2=[ind2 length(ind)-1];
elseif(length(ind2)>length(ind1));
    ind1=[1 ind1];    
else
    ind1=[1 ind1];    
    ind2=[ind2 length(ind)-1];
end
% find length of all segments
segmentlength=ind2-ind1+1;
% check minimum segment length 
numseg=min(sum(segmentlength>=minlength),numseg);

if numseg==0
  segmentlength=0;indrange=[];unitvec=[];mb=[];x{1}=[];y{1}=[];return
end

% check how many segments
[sL,sID]=sort(segmentlength,'Descend');

% plotting
if fignum>0
    figure(fignum);
    plot(pt(:,1),pt(:,2),'x',pt(ind,1),pt(ind,2),'o','linewidth',1);
    hold on
end
% 
for i=1:min(numseg,length(segmentlength))
    startind=ind(ind1(sID(i)));
    endind=ind(ind2(sID(i))-1)+1;
    indrange{i}=(startind:endind); 
    mb=polyfit(pt(indrange{i},1),pt(indrange{i},2),1);
    x{i} = [min(pt(indrange{i},1)) max(pt(indrange{i},1))];
    y{i} = mb(1)*x{i} + mb(2);
    unitvec{i}=[1 mb(1)]/norm([1 mb(1)]);
    if fignum>0
        plot(x{i},y{i},'-k','linewidth',4);
        %plot(pt(indrange{i},1),pt(indrange{i},2),'-^','linewidth',2)
    end
end
hold off

end