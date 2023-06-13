function [t,x,u] = simpendulum(K,d,epsilon,init)
E = 2;
[nu,nx,nt] = size(K);
t  = linspace(0,1,nt);
x  = zeros(nx,nt);
u  = zeros(nu,nt);
x(:,1) = init;
dt = 1/(nt-1);
dw = randn(1,nt);
dw = sqrt(dt)*dw;
for i = 1:nt-1
   u(:,i)   = K(:,:,i)*x(:,i)+d(1,i);
   x(:,i+1) = x(:,i)+ dt*([x(2,i);E*sin(x(1,i))+K(:,:,i)*x(:,i)+d(1,i)])+[0;sqrt(epsilon)*dw(i)];
end
u(:,end) = u(:,end-1);
end