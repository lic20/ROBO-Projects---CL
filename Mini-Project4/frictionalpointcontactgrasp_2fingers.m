clear all;close all;


%q=pi/4;
%q=pi/3.9;
%q=pi/6;
q=pi/8;
%q=pi/16;
%q=pi/100;
x1=-1;y1=-0.5;
x2=1;y2=0;
x3=1;y3=-1;
x4=-1;y4=1;
%x3=0;x4=0;
%y1=0;y2=0;
%y1=.2;y2=-.2;

f(:,1)=[1;0];
f(:,2)=[-1;0];
f(:,3)=[0;1];
f(:,4)=[0;-1];
pC(:,1)=[x1;y1];
pC(:,2)=[x2;y2];
pC(:,3)=[x3;y3];
pC(:,4)=[x4;y4];

S=[0 -1;1 0];
tau(1)=(S*pC(:,1))'*rot2(q)*f(:,1);
tau(2)=(S*pC(:,1))'*rot2(-q)*f(:,1);
tau(3)=(S*pC(:,2))'*rot2(q)*f(:,2);
tau(4)=(S*pC(:,2))'*rot2(-q)*f(:,2);
tau(5)=(S*pC(:,3))'*rot2(q)*f(:,3);
tau(6)=(S*pC(:,3))'*rot2(-q)*f(:,3);
tau(7)=(S*pC(:,4))'*rot2(q)*f(:,4);
tau(8)=(S*pC(:,4))'*rot2(-q)*f(:,4);

G1(:,1)=[tau(1);rot2(q)*f(:,1)];
G1(:,2)=[tau(2);rot2(-q)*f(:,1)];
G1(:,3)=[tau(3);rot2(q)*f(:,2)];
G1(:,4)=[tau(4);rot2(-q)*f(:,2)];
G1(:,5)=[tau(5);rot2(q)*f(:,3)];
G1(:,6)=[tau(6);rot2(-q)*f(:,3)];
G1(:,7)=[tau(7);rot2(q)*f(:,4)];
G1(:,8)=[tau(8);rot2(-q)*f(:,4)];
G1=G1(:,1:4);
m2=size(G1,2);

% minimizing ||G * x||^2, x>0
[X,FVAL,EXITFLAG,OUTPUT] = quadprog(G1'*G1,zeros(m2,1),-eye(m2),-ones(m2,1));
% minimizing ||x||^2 subject to G * x = 0, x>0
[X1,FVAL1,EXITFLAG,OUTPUT] = quadprog(eye(m2),zeros(m2,1),-eye(m2),-ones(m2,1),G1,zeros(3,1));

if FVAL>.1;disp('^^^ NO FORCE CLOSURE GRASP ^^^');return;end
if isempty(X1);disp('^^^ NO FORCE CLOSURE GRASP ^^^');return;end

disp('*******************')
fprintf('minimum ||G x ||, x>0: %g, %g \n',FVAL,norm(G1*X1)^2);
disp('*******************')
Gsym=sym(G1)
nullGsym=null(Gsym);

xi_vec=sym(zeros(size(nullGsym,2),1));
for i=1:size(nullGsym,2)
    x_i=['xi',num2str(i)];
    eval(['syms ',x_i]);
    eval(['xi_vec(i)=',x_i,';']);
end
disp(null(Gsym)*xi_vec);


%
% example of how to use the null space of G to satisfy force closure
%
etaC = randn(3,1);
disp('without squeeze pinv(G)*etaC:')
disp(pinv(G1)*etaC);
disp('with squeeze pinv(G)*etaC + alpha*X, X\in null(G):')
fun = @(x,eta_m,eta_s) -min(min(eta_m+x*eta_s),0);  % The parameterized function.
alpha = fminbnd(@(x) fun(x,pinv(G1)*etaC,X),0,1);
alpha1 = fminbnd(@(x) fun(x,pinv(G1)*etaC,X1),0,1);
eta=pinv(G1)*etaC+alpha*X;
f_contact=[rot2(q)*f(:,1)*eta(1)+rot2(-q)*f(:,1)*eta(2),...
    rot2(q)*f(:,2)*eta(3)+rot2(-q)*f(:,2)*eta(4)];
eta1=pinv(G1)*etaC+alpha1*X1;
f1_contact=[rot2(q)*f(:,1)*eta1(1)+rot2(-q)*f(:,1)*eta1(2),...
    rot2(q)*f(:,2)*eta1(3)+rot2(-q)*f(:,2)*eta1(4)];    
disp([eta,eta1]);
disp('**** Checking the contact forces generate the same eta_C')
disp('[etaC G*eta G*eta1]')
disp([etaC G1*eta G1*eta1]);

%
% visualize
%
figure(1);
p0C=[2;2];w2=1;h2=1;

pcorner(:,1)=p0C+[w2;h2];
pcorner(:,2)=p0C+[w2;-h2];
pcorner(:,3)=p0C+[-w2;-h2];
pcorner(:,4)=p0C+[-w2;h2];
pcorner(:,5)=p0C+[w2;h2];

p01=p0C+[x1;y1];
p02=p0C+[x2;y2];
p03=p0C+[x3;y3];
p04=p0C+[x4;y4];

f_cone=[rot2(q)*f(:,1),rot2(-q)*f(:,1),...
    rot2(q)*f(:,2),rot2(-q)*f(:,2),...
    rot2(q)*f(:,3),rot2(-q)*f(:,3),...
    rot2(q)*f(:,4),rot2(-q)*f(:,4)]; 
f_cone=f_cone./vecnorm(f_cone)/2;

plot(pcorner(1,:),pcorner(2,:),':','linewidth',2);hold on;
f_contact=f_contact./vecnorm(f_contact);
quiver(p01(1),p01(2),f_contact(1,1),f_contact(2,1),'linewidth',2);
quiver(p02(1),p02(2),f_contact(1,2),f_contact(2,2),'linewidth',2);
%quiver(p03(1),p03(2),f_contact(1,3),f_contact(2,3));
%quiver(p04(1),p04(2),f_contact(1,4),f_contact(2,4));
f1_contact=f1_contact./vecnorm(f1_contact);
quiver(p01(1),p01(2),f1_contact(1,1),f1_contact(2,1),'--','linewidth',2);
quiver(p02(1),p02(2),f1_contact(1,2),f1_contact(2,2),'--','linewidth',2);
%quiver(p03(1),p03(2),f1_contact(1,3),f1_contact(2,3),'--');
%quiver(p04(1),p04(2),f1_contact(1,4),f1_contact(2,4),'--');

plot([p01(1) p01(1)+f_cone(1,1)],[p01(2) p01(2)+f_cone(2,1)],'k:','linewidth',2);
plot([p01(1) p01(1)+f_cone(1,2)],[p01(2) p01(2)+f_cone(2,2)],'k:','linewidth',2);
plot([p02(1) p02(1)+f_cone(1,3)],[p02(2) p02(2)+f_cone(2,3)],'k:','linewidth',2);
plot([p02(1) p02(1)+f_cone(1,4)],[p02(2) p02(2)+f_cone(2,4)],'k:','linewidth',2);
%plot([p03(1) p03(1)+f_cone(1,5)],[p03(2) p03(2)+f_cone(2,5)],'k:','linewidth',2);
%plot([p03(1) p03(1)+f_cone(1,6)],[p03(2) p03(2)+f_cone(2,6)],'k:','linewidth',2);
%plot([p04(1) p04(1)+f_cone(1,7)],[p04(2) p04(2)+f_cone(2,7)],'k:','linewidth',2);
%plot([p04(1) p04(1)+f_cone(1,8)],[p04(2) p04(2)+f_cone(2,8)],'k:','linewidth',2);

f_C=etaC(2:3)/norm(etaC(2:3))/1.5;
quiver(p0C(1),p0C(2),f_C(1),f_C(2),'r','linewidth',3);

theta=(0:pi/4/10:pi/4);
plot(p0C(1)+.2*sin(theta),p0C(2)+.2*cos(theta),'r:','linewidth',3);

axis([0.5 3.5 0.5 3.5]);
axis('square');
hold off


%
% rot2 function
% 
function R=rot2(q)
    R=[cos(q) -sin(q); sin(q) cos(q)];
end

