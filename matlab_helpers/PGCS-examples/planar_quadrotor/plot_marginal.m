function [] = plot_marginal(m0, zk, Sig0_xy, Sk_xy, nt, sig, step_size)
%PLOT_ELLIP Summary of this function goes here
%   Detailed explanation goes here
n  = 2;
t  = linspace(0,sig,nt);
n3=100;
theta=linspace(0,2*pi,n3);
circlepara=[cos(theta);sin(theta)];
tellips=zeros(n,n3,step_size);
tellips(:,:,1)= sqrtm(Sig0_xy)*circlepara+m0(1:2)*ones(1,n3);
for i=1:step_size-1
    indx = floor(nt/step_size)*i;
    tellips(:,:,i+1)=3*sqrtm(Sk_xy(:,:,1+indx))*circlepara+zk(1:2,1+indx)*ones(1,n3);
end
X=zeros(step_size,n3);Y=zeros(step_size,n3);Z=zeros(step_size,n3);
for i=1:step_size
    X(i,:)=t(1+floor(nt/step_size)*(i-1))*ones(1,n3);
    Y(i,:)=tellips(1,:,i);
    Z(i,:)=tellips(2,:,i);
end

cstring='grbcmk';
hold on; 
% grid on;
surf(X,Y,Z,'FaceColor','blue','EdgeColor','none');
alpha(0.1);
view(-30,30);
end

