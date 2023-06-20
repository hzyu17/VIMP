function [K,d] = linearCov(A,a,B,epsilon,Q,r,m0,Sig0,m1,Sig1,sig)
% sig: time scaling
[nx,nu,nt] = size(B);
dt         = sig/(nt-1);
% t          = linspace(0,1,nt);
I          = eye(nx);
M          = zeros(2*nx,2*nx,nt);
for i = 1:nt
    M(:,:,i) = [A(:,:,i) -B(:,:,i)*B(:,:,i)';-Q(:,:,i) -A(:,:,i)'];
end
Phi   = eye(2*nx);
for i = 1:nt-1
    Phi_tild = Phi+dt.*(M(:,:,i)*Phi);
    Phi  = Phi+ dt.* ( (M(:,:,i)*Phi) + (M(:,:,i+1)*Phi_tild) ) / 2;
end
Phi12 = Phi(1:nx,nx+1:2*nx);
Phi11 = Phi(1:nx,1:nx);

s     = zeros(2*nx,1);
for i = 1:nt-1
    s_tild = s+dt.*(M(:,:,i)*s+[a(:,i);-r(:,i)]);
    s = s+dt.*( (M(:,:,i)*s+[a(:,i);-r(:,i)]) + (M(:,:,i+1)*s_tild+[a(:,i+1);-r(:,i+1)]) ) / 2;
end

lambda0 = Phi12\(m1-Phi11*m0-s(1:nx));
Xl = zeros(2*nx,nt);
Xl(:,1) = [m0;lambda0];
for i = 1:nt-1
    temp = Xl(:,i);
    Xtild = temp + dt.*(M(:,:,i)*temp+[a(:,i);-r(:,i)]);
   Xl(:,i+1) = Xl(:,i)+dt.*((M(:,:,i)*temp+[a(:,i);-r(:,i)]) + (M(:,:,i+1)*Xtild+[a(:,i+1);-r(:,i+1)]) ) / 2;
end
bx = Xl(1:nx,:);
bl = Xl(nx+1:2*nx,:);
v  = zeros(nu,nt);
for i = 1:nt
    v(:,i) = -B(:,:,i)'*bl(:,i);
end
Pi = zeros(nx,nx,nt);
% inv_Phi12 = inv(Phi12);
Pi0 = epsilon*inv(Sig0)/2 - Phi12\Phi11-Sig0^(-1/2) * (epsilon^2* I/4+...
    Sig0^0.5*(Phi12\Sig1)*(Phi12'\(Sig0^0.5)))^(1/2) * Sig0^(-1/2);
Pi(:,:,1) = (Pi0+Pi0')./2;
% B1 = zeros(6,6);
% B1(5,5) = 1;
% B1(6,6) = 1;
for i = 1:nt-1
    temp=Pi(:,:,i);
    Pi_tild = temp-dt.*(A(:,:,i)'*temp+temp*A(:,:,i)-temp*B(:,:,i)*B(:,:,i)'*temp+Q(:,:,i));
    Pi(:,:,i+1)=temp-dt.*( (A(:,:,i)'*temp+temp*A(:,:,i)-temp*B(:,:,i)*B(:,:,i)'*temp+Q(:,:,i))...
        + (A(:,:,i+1)'*Pi_tild+Pi_tild*A(:,:,i+1)-Pi_tild*B(:,:,i+1)*B(:,:,i+1)'*Pi_tild+Q(:,:,i+1)) ) / 2;
end
K = zeros(nu,nx,nt);
d = zeros(nu,nt);
for i = 1:nt
    K(:,:,i) = -B(:,:,i)'*Pi(:,:,i);
    d(:,i)   = v(:,i)+B(:,:,i)'*Pi(:,:,i)*bx(:,i);
end
end