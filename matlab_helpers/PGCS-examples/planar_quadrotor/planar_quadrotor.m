function [t,x,u] = planar_quadrotor(K,d,epsilon,init,sig)

[nu, nx, nt] = size(K);
t  = linspace(0, sig, nt);
x  = zeros(nx, nt);
u  = zeros(nu, nt);
x(:,1) = init;
dt = sig / (nt-1);

g=9.81;
m=0.486;
J=0.00383;
l=0.25;

% B = [ 0         0;
%          0         0;
%          0         0;
%          0         0;
%         1/m    1/m;
%         l/J  -l/J];

B = 10.*[ 0, 0; 
                0, 0; 
                0, 0; 
                0, 0; 
                1/sqrt(2), 1/sqrt(2); 
                1/sqrt(2), -1/sqrt(2)];

for i = 1:nt-1
   u(:,i)   = K(:,:,i)*x(:,i)+d(:,i);
   x(:,i+1) = x(:,i)+ dt.*(B*u(:,i) + ...
                                [ x(4,i)*cos(x(3,i)) - x(5,i)*sin(x(3,i));
                                  x(4,i)*sin(x(3,i)) + x(5,i)*cos(x(3,i));
                                  x(6,i);
                                  x(5,i)*x(6,i)-g*sin(x(3,i));
                                  -x(4,i)*x(6,i)-g*cos(x(3,i))
                                  0]) + B*[sqrt(epsilon*dt)*randn; sqrt(epsilon*dt)*randn];
end
u(:,end) = u(:,end-1);
end