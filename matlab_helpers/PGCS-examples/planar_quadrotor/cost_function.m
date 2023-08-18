function total_cost = cost_function(K,d,epsilon,init,sig,Q,zk)
% COST_FUNCTION Compute the total cost of a given optimal feedback policy
% (K, d). Computed using the empirical expectation of samples.

% dynamics definitions
cd = 0.005;
[nu,nx,nt] = size(K);
t  = linspace(0,sig,nt);

num_samples = 500;
total_cost = 0;
% each sampled trajectory
for i_sample = 1:num_samples
x  = zeros(nx,nt);
u  = zeros(nu,nt);
x(:,1) = init;
dt = sig/(nt-1);
dw = randn(1,nt);
dw = sqrt(dt)*dw;
B = [0 0; 0 0; 1 0; 0 1];
    for i = 1:nt-1
       u(:,i)   = K(:,:,i)*x(:,i)+d(:,i);
       x(:,i+1) = x(:,i)+ dt.*(B*u(:,i) + ...
                                    [ x(3,i);
                                      x(4,i); 
                                    -cd*sqrt(x(3,i)^2+x(4,i)^2)*x(3,i); 
                                    -cd*sqrt(x(3,i)^2+x(4,i)^2)*x(4,i)])...
                                + [0;0;sqrt(epsilon)*dw(i);sqrt(epsilon)*dw(i)];
    
       total_cost = total_cost + ((x(:,i)-zk(:,i))'*Q(:,:,i)*(x(:,i)-zk(:,i)) + u(:, i)'*u(:, i)/2) * dt;
    end
end
total_cost = total_cost / num_samples
end

