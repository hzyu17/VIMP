nx = 2;
nu = 1;
nt = 10001;
dt = 1/(nt-1);
t  = linspace(0,1,nt);
A1 = [0 1;0 0];
B1 = [0;1];
Q1 = eye(2);
A  = A1(:)*ones(1,nt);
A  = reshape(A,[nx,nx,nt]);
B  = B1(:)*ones(1,nt);
B  = reshape(B,[nx,nu,nt]);
Q  = Q1(:)*ones(1,nt);
Q  = reshape(Q,[nx,nx,nt]);

a  = ones(nx,nt);
r  = zeros(nx,nt);

m0 = [3.14;0];
m1 = [0;0];

epsilon = 0.1;
Sig0    = [0.1 0;0 0.5];
% Sig1    = [0.6 0.1;0.1 0.3];
Sig1    = [0.01 0;0 0.01];



%%
A0 = A;
a0 = zeros(nx,nt);
As = A0;
as = a0;
nAs = A0;
nas = a0;
iter = 0;
eta = 0.1;
y   = zeros(5,iter);
err = 1;
tic
while err > 1e-5
    iter = iter+1;
    [Sk,zk]       = Szk(As,B,as,epsilon,m0,Sig0);
    [hAk,hak,nTr] = linearAa(Sk,zk,As);
    Aprior   = (eta/(1+eta)).*As+(1/(1+eta)).*hAk;
    aprior   = (eta/(1+eta)).*as+(1/(1+eta)).*hak;
    y(1,k)   = Aprior(1,2,4);
    y(2,k)   = As(2,1,9);
    y(3,k)   = As(2,2,10);
    y(4,k)   = as(1,11);
    y(5,k)   = as(2,12);
    [Qk, rk] = Qr(As,as,hAk,hak,nTr,eta,B);
    [K,d] = linearCov(Aprior,aprior,B,epsilon,Qk,rk,m0,Sig0,m1,Sig1);
    for i = 1:nt
        nAs(:,:,i) = Aprior(:,:,i)+B(:,:,i)*K(:,:,i);
        nas(:,i) = aprior(:,i)+B(:,:,i)*d(:,i);
    end
    err = norm(nAs(:)-As(:))/norm(nAs(:))+norm(nas(:)-as(:))/norm(nas(:));
    As = nAs;
    as = nas;
end
toc
iter

%%
dA  = As-hAk;
da  = as-hak;
Ks  = dA(2,:,:);
ds  = da(2,:);
% % init = randn(2,1);
% init = [0;0];
% init = Sig0^(1/2)*init+m0;
% % [t,x] = simpendulum(Ks,ds,epsilon,init);
% [t,x,u] = simpendulum(Ks,ds,0,init);
% figure(2),plot3(t,x(1,:),x(2,:));

%%
n  = 2;
n1 = nt;
n2=6;
x0=zeros(n,n2);
x1=zeros(n,n2);
n3=100;
theta=linspace(0,2*pi,n3);
circlepara=[cos(theta);sin(theta)];
n4=21;
tellips=zeros(n,n3,n4);
tellips(:,:,1)=3*sqrtm(Sig0)*circlepara+m0*ones(1,n3);
for i=1:n4-1
    tellips(:,:,i+1)=3*sqrtm(Sk(:,:,1+(n1-1)/(n4-1)*i))*circlepara+zk(:,1+(n1-1)/(n4-1)*i)*ones(1,n3);
end
X=zeros(n4,n3);Y=zeros(n4,n3);Z=zeros(n4,n3);
for i=1:n4
    X(i,:)=t(1+(n1-1)/(n4-1)*(i-1))*ones(1,n3);
    Y(i,:)=tellips(1,:,i);
    Z(i,:)=tellips(2,:,i);
end
cstring='grbcmk';
%cstring='cccccc';
figure(1),hold on;
%mesh(X,Y,Z);
surf(X,Y,Z,'FaceColor','blue','EdgeColor','none');
alpha(0.1);
view(-30,30);
% for i=1:n4
%     plot3(t(1+(n1-1)/(n4-1)*(i-1))*ones(1,n3),tellips(1,:,i),tellips(2,:,i));
% end

%%

figure(2),hold on;
% figure(3),hold on;
for j=1:n2
% dw=randn(nu,n1)*ddt;
% x=zeros(n,n1);
% x(:,1)=sqrtm(Sig0)*randn(n,1);
% u=zeros(nu,n1);
% for i=1:n1-1
%     temp=x(:,i);
%     x(:,i+1)=temp+dt*(A*temp-B*B'*Qinv(:,:,i)*temp)+B*dw(:,i);
% end
% for i=1:n1
%     u(:,i)=-B'*Qinv(:,:,i)*x(:,i);
% end
init = randn(2,1);
% init = [0;0];
init = Sig0^(1/2)*init+m0;
% [t,x] = simpendulum(Ks,ds,epsilon,init);
[~,x,u] = simpendulum(Ks,ds,epsilon,init);
x0(:,j)=x(:,1);
x1(:,j)=x(:,n1);
figure(1),plot3(t,x(1,:),x(2,:),cstring(mod(j,6)+1),'LineWidth',2);
%figure(1),plot3(t,x(1,:),x(2,:),'color',[0.3 0.3 0.3]);
figure(2),plot(t,u(1,:),cstring(mod(j,6)+1),'LineWidth',2);
% figure(3),plot(x(1,:),x(2,:),cstring(mod(j,6)+1));
end
figure(1),hold off;
figure(2),hold off;
% figure(3),hold off;
figure(1),
% axis([0 1 -5 5 -4 4])
xlabel('Time $t$','Interpreter','latex'),ylabel('Position $x$','Interpreter','latex');
zlabel('Velocity $v$','Interpreter','latex');
set(gca,'fontsize',16);
figure(2),
xlabel('Time $t$','Interpreter','latex'),ylabel('Control input $u$','Interpreter','latex');
set(gca,'fontsize',16);
% figure(3),
% xlabel('Position $x$','Interpreter','latex');ylabel('Velocity $v$','Interpreter','latex');
% 

%%
x = [100 200 500 1000 1500 2000];
t = [0.039 0.064 0.136 0.254 0.370 0.510];
figure(3), plot(x,t,'LineWidth',2);
xlabel('Number of discretization steps','Interpreter','latex'),ylabel('Running time','Interpreter','latex');
set(gca,'fontsize',16);
