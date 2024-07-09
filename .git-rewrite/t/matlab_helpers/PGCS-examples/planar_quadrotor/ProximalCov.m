function [As,as] = ProximalCov(A0,a0,B,epsilon,eta,iter,m0,Sig0,m1,Sig1)
[nx,nu,nt] = size(B);
As = A0;
as = a0;
for k = 1:iter
    [Sk,zk]       = Szk(As,B,as,epsilon,m0,Sig0);
    [hAk,hak,nTr] = linearAa(Sk,zk,As);
    Aprior   = (eta/(1+eta)).*As+(1/(1+eta)).*hAk;
    aprior   = (eta/(1+eta)).*as+(1/(1+eta)).*hak;
    [Qk, rk] = Qr(As,as,hAk,hak,nTr,eta,B);
    [K,d] = linearCov(Aprior,aprior,B,epsilon,Qk,rk,m0,Sig0,m1,Sig1);
    for i = 1:nt
        As(:,:,i) = Aprior(:,:,i)+B(:,:,i)*K(:,:,i);
        as(:,i) = aprior(:,i)+B(:,:,i)*d(:,i);
    end
end
end
