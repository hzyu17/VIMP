function [Qk, rk] = QrV(As,as,hAk,hak,nTr,eta,B, pinvBBT, Q,zk)
% Compute (Qk, rk) with a fixed quadratic V = x'Qx.

[nx,nu,nt] = size(B);
Qk  = zeros(nx,nx,nt);
rk  = zeros(nx,nt);
for i = 1:nt
    Qk(:,:,i) = 2*eta/(1+eta).*Q(:,:,i) + eta/(1+eta)/(1+eta).*(As(:,:,i)-hAk(:,:,i))'*pinvBBT*(As(:,:,i)-hAk(:,:,i));
    rk(:,i)   = - eta/(1+eta).*(Q(:,:,i)*zk(:, i)) ...
                + eta/2/(1+eta).*nTr(:,i) ...
                + eta/(1+eta)^2.*(As(:,:,i)-hAk(:,:,i))'*pinvBBT*(as(:,i)-hak(:,i));
end
end