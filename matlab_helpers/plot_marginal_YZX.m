function [] = plot_marginal_YZX(m0, zk, Sig0_xy, Sk_xy, nt, sig, num_drawings)
%PLOT_ELLIP Summary of this function goes here
%   Detailed explanation goes here
n  = 2;
t  = linspace(0,sig,nt);
n3=100;
theta=linspace(0,2*pi,n3);
circlepara=[cos(theta);sin(theta)];
tellips=zeros(n,n3,num_drawings);
% tellips=zeros(n,n3,nt);
tellips(:,:,1)= sqrtm(Sig0_xy)*circlepara+m0(1:2)*ones(1,n3);
for i=1:nt-1
    indx = floor(nt/num_drawings)*i;
    tellips(:,:,i+1)=3*sqrtm(Sk_xy(:,:,1+indx))*circlepara+zk(1:2,1+indx)*ones(1,n3);
%     tellips(:,:,i+1)=3*sqrtm(Sk_xy(:,:,1+i))*circlepara+zk(1:2,1+i)*ones(1,n3);
end
X=zeros(num_drawings,n3);Y=zeros(num_drawings,n3);Z=zeros(num_drawings,n3);
for i=1:num_drawings
    X(i,:)=t(1+floor(nt/num_drawings)*(i-1))*ones(1,n3);
    Y(i,:)=tellips(1,:,i);
    Z(i,:)=tellips(2,:,i);
end

cstring='grbcmk';
hold on; 
% grid on;
surf(Y,Z,X,'FaceColor','blue','EdgeColor','none','FaceAlpha', 0.1);
% alpha(0.1);
view(-30,30);
end

