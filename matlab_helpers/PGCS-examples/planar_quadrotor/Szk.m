function [Sk,zk] = Szk(A,B,a,epsilon,m0,Sig0, sig)
[nx,nu,nt] = size(B);
dt         = sig/(nt-1);
zk         = zeros(nx,nt);
Sk         = zeros(nx,nx,nt);
zk(:,1)    = m0;
Sk(:,:,1)  = Sig0;
for i = 1:nt-1
    zk1   = zk(:,i) + dt*(A(:,:,i)*zk(:,i) + a(:,i));    
    zk(:,i+1) = zk(:,i) + dt*(A(:,:,i)*zk(:,i) + a(:,i) + A(:,:,i+1)*zk1 + a(:,i+1)) / 2;
    
    temp        = Sk(:,:,i);
    Sk1 = temp + dt*(A(:,:,i)*temp + temp*A(:,:,i)' + epsilon*(B(:,:,i)*B(:,:,i)'));
    
    Sk(:,:,i+1) = temp + dt*((A(:,:,i)*temp + temp*A(:,:,i)' + epsilon*(B(:,:,i)*B(:,:,i)')) + ...
                             A(:,:,i+1)*Sk1 + Sk1*A(:,:,i+1)' + epsilon*(B(:,:,i+1)*B(:,:,i+1)'))/2;
end
end