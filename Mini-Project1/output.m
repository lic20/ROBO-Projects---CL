%
% simulate sensors
%

% range sensors from UWB sensors

function y=output(q,pL,pB,N_scan,utrue,vcov,robot,colobj)

N_range=size(pL,2);N_bearing=size(pB,2);N_odo=2;
y=zeros(N_range+N_odo+N_bearing+N_scan,1);

% height of the sensor on robot
rz=robot.Z/2;
ez=[0;0;1];

% position and orientation
pR=[q(1:2);rz];
theta=q(3);

% % UWB (local GPS)
sigma=sqrt(vcov(1:N_range));
y(1:N_range)=range_sensor(pL,pR,sigma);

% odometry and IMU
sigma=sqrt(vcov(N_range+1:N_range+N_odo));
y(N_range+1:N_range+2)=utrue+randn(2,1).*sigma;
    
% bearing sensors 
pB_pR=(pB-pR)-ez*ez'*(pB-pR); % remove the z component
sigma=sqrt(vcov(N_range+N_odo+1:N_range+N_odo+N_bearing));
y(N_range+N_odo+1:N_range+N_odo+2)=...
    bearing_sensor(pB_pR,rot(ez,theta)*[1;0;0],[0;0;1],sigma);

% scanner 
sigma=sqrt(vcov(N_range+N_odo+N_bearing+1:N_range+N_odo+N_bearing+N_scan));
y(N_range+N_odo+N_bearing+1:N_range+N_odo+N_bearing+N_scan)=...
    scanpattern(q,robot,colobj,N_scan,1)'+randn(size(sigma)).*sigma;
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

