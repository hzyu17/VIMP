function [Qk, rk] = Qr(As,as,hAk,hak,nTr,eta,B)
[nx,nu,nt] = size(B);
Qk  = zeros(nx,nx,nt);
rk  = zeros(nx,nt);
for i = 1:nt
    temp1 = As(:,:,i);
    temp2 = hAk(:,:,i);
    temp3 = as(:,i);
    temp4 = hak(:,i);
    Qk(:,:,i) = eta/(1+eta)^2*(temp1-temp2)'*B(:,:,i)*B(:,:,i)'*(temp1-temp2);
    rk(:,i)   = eta/(1+eta)^2*(temp1-temp2)'*B(:,:,i)*B(:,:,i)'*(temp3-temp4)+ eta/2/(1+eta)*nTr(:,i);
end
end