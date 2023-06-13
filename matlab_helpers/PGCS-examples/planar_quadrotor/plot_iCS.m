function [t,x,u] = plot_iCS(iCS_data, init, epsilon, sig)
%UNTITLED Summary of this function goes here

uhat = iCS_data.uhat;
xhat = iCS_data.xhat;
ubar = iCS_data.ubar;
xbar = iCS_data.xbar;
K = iCS_data.K;

cd = 0.005;
[nu,nx,nt] = size(K);
t  = linspace(0,sig,nt);
x  = zeros(nx,nt);
u  = zeros(nu,nt);
x(:,1) = init;
dt = sig/(nt-1);
dw = randn(1,nt);
dw = sqrt(dt)*dw;
B = [0 0; 0 0; 1 0; 0 1];
for i = 1:nt-1
   u(:,i)   = ubar(:,i) + uhat(:,i);
   x(:,i+1) = x(:,i)+ dt.*(B*u(:,i) + ...
                                [ x(3,i);
                                  x(4,i); 
                                -cd*sqrt(x(3,i)^2+x(4,i)^2)*x(3,i); 
                                -cd*sqrt(x(3,i)^2+x(4,i)^2)*x(4,i)])...
                            + [0;0;sqrt(epsilon)*dw(i);sqrt(epsilon)*dw(i)];
end
u(:,end) = u(:,end-1);
end