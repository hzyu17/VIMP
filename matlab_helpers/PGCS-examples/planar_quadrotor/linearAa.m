function [hAk,hak,nTr] = linearAa(Sk,zk,A)
% assume V = 0
% hAk  = A;
% hak  = a;
% Qk   = Q;
% rk   = r;
E = 2;
[nx,~,nt] = size(Sk);
hAk = zeros(nx,nx,nt);
hak = zeros(nx,nt);
nTr = zeros(nx,nt);

for i = 1:nt
    hAk(:,:,i) = [0 1; E*cos(zk(1,i)) 0];
    hak(:,i) = [zk(2,i);E*sin(zk(1,i))] - hAk(:,:,i)*zk(:,i);
    nTr(:,i) = [-2*E^2*Sk(1,1,i)*cos(zk(1,i))*sin(zk(1,i))+2*E*Sk(1,2,i)*A(2,2,i)*sin(zk(1,i)); 0];
    
end
end