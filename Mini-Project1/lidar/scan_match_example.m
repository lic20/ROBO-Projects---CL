%
% use the stored robot path to show scan match for two consecutive scans
%
% modify scan_i scan_j below to pick out the scan lines
% M = # of scans
%
close all
%scan_i=50;scan_j=53;
scan_i=fix(rand*length(q));scan_j=scan_i+1;
%scan_i=184;scan_j=scan_i+1;

% # of scan lines
M=200;

% generate scans 
l_i=scanpattern(q(:,scan_i),robot,colobj,M,1);
l_j=scanpattern(q(:,scan_j),robot,colobj,M,1);

% 
lidarscan_i=lidarScan(l_i,linspace(-pi/2,pi/2,M)+pi/2);
lidarscan_j=lidarScan(l_j,linspace(-pi/2,pi/2,M)+pi/2);

% 
T_ij=matchScans(lidarscan_j,lidarscan_i);
lidarscan_j_trans=transformScan(lidarscan_j,T_ij);
%

figure(10);plot(lidarscan_i.Cartesian(:,1),lidarscan_i.Cartesian(:,2),'k.',...
    lidarscan_j.Cartesian(:,1),lidarscan_j.Cartesian(:,2),'r.');
title('scan pattern comparison');

%

figure(11);plot(lidarscan_i.Cartesian(:,1),...
    lidarscan_i.Cartesian(:,2),'k.',...
    lidarscan_j_trans.Cartesian(:,1),lidarscan_j_trans.Cartesian(:,2),'r.');
title('scan pattern comparison (with correction)');

% 
disp('Transform from scan i to scan j vs. actual transform');
disp([T_ij;diff(q(:,[scan_i,scan_j])')]);

% occupancy map (assuming perfect robot pose)

map = occupancyMap(10,10,20);
insertRay(map,q(:,scan_i),lidarscan_i,20);
figure(50);show(map);view([0,-90]);
insertRay(map,q(:,scan_j),lidarscan_j,20);
figure(51);show(map);view([0,-90]);