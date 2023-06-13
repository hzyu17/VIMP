nx = 2;
nu = 1;
nt = 1001;
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

epsilon = 0.01;
Sig0    = 0.5*[0.2 0;0 0.1];
% Sig1    = [0.6 0.1;0.1 0.3];
Sig1    = [0.01 0;0 0.01];

%%
[K,d] = linearCov(A,a,B,epsilon,Q,r,m0,Sig0,m1,Sig1);

m          = zeros(nx,nt);
Sig        = zeros(nx,nx,nt);
m(:,1)     = m0;
Sig(:,:,1) = Sig0;
for i = 1:nt-1
    A2        = A(:,:,i);
    B2        = B(:,:,i);
    K2        = K(:,:,i);
    d2        = d(:,i);
    temp0     = m(:,i);
    m(:,i+1)  = temp0+dt*((A2+B2*K2)*temp0+a(:,i)+B2*d2);
    temp=Sig(:,:,i);
    Sig(:,:,i+1)=temp+dt*((A2+B2*K2)*temp+temp*(A2+B2*K2)'+epsilon*(B2*B2'));
end

%%
Ak1 = zeros(nx,nx,nt);
ak1 = zeros(nx,nt);
for i = 1:nt
    Ak1(:,:,i) = A(:,:,i)+B(:,:,i)*K(:,:,i);
    ak1(:,i) = a(:,i)+B(:,:,i)*d(:,i);
end
[Sk,zk] = Szk(Ak1,B,ak1,epsilon,m0,Sig0);

%%
A0 = A;
a0 = zeros(nx,nt);
As = A0;
as = a0;
iter = 100;
eta = 0.1;
y   = zeros(5,iter);
for k = 1:iter
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
        As(:,:,i) = Aprior(:,:,i)+B(:,:,i)*K(:,:,i);
        as(:,i) = aprior(:,i)+B(:,:,i)*d(:,i);
    end
end

figure(1),plot(1:iter,y(1:5,:));

%%
dA  = As-hAk;
da  = as-hak;
Ks  = dA(2,:,:);
ds  = da(2,:);
% init = randn(2,1);
init = [0;0];
init = Sig0^(1/2)*init+m0;
% [t,x] = simpendulum(Ks,ds,epsilon,init);
[t,x,u] = simpendulum(Ks,ds,0,init);
figure(2),plot3(t,x(1,:),x(2,:));

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
figure(2),
xlabel('Time $t$','Interpreter','latex'),ylabel('Control input $u$','Interpreter','latex');
% figure(3),
% xlabel('Position $x$','Interpreter','latex');ylabel('Velocity $v$','Interpreter','latex');
% 


