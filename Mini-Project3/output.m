%
% simulate sensors
%

% range sensors from UWB sensors

function y=output(q,pL,pB,vcov)

N_range=size(pL,2);N_bearing=size(pB,2);
y=zeros(N_range+N_bearing,1);

% height of the sensor on robot
ez=[0;0;1];

% position and orientation
pR=[q(1:2);0];
theta=0;

% % UWB (local GPS)
sigma=sqrt(vcov(1:N_range));
y(1:N_range)=range_sensor(pL,pR,sigma);

    
% bearing sensors 
pB_pR=(pB-pR)-ez*ez'*(pB-pR); % remove the z component
sigma=sqrt(vcov(N_range+1:N_range+N_bearing));
y(N_range+1:N_range+2)=...
    bearing_sensor(pB_pR,rot(ez,theta)*[1;0;0],[0;0;1],sigma);

end

function y=range_sensor(pL,pR,sigma)

y=vecnorm(pL-pR)'+randn(size(sigma)).*sigma;

end

function y=bearing_sensor(pB,k_heading,k_vertical,sigma)

    k_heading=k_heading/norm(k_heading);

    for i=1:size(pB,2)

        y(i)=subprob1(k_vertical,k_heading,pB(:,i)/norm(pB(:,i))) + ...
            randn*sigma(i);
    end

end

function khat = hat(k)
  
  khat=[0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];

end

