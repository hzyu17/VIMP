%% implement the Gaussian interpolation
clear all
clc

dim_conf = 2; dim_theta = 4;
n_interp = 100;
delta_t = 0.1;

Qc = eye(dim_conf);

mu1 = [1; 1; 0.3; 0.3];
mu2 = [3; 3; 0.4; 0.4];

cov1 = [1,0;
    0,1].*0.01;
cov2 = [1,0;
    0,1].*0.01;

rng('default')  % For reproducibility
conf1 = mvnrnd(mu1(1:2), cov1,1)';
conf2 = mvnrnd(mu2(1:2), cov2,1)';

% conf1 = mu1(1:2);
% conf2 = mu2(1:2);

vel1 = mvnrnd(mu1(3:4), cov1, 1)';
vel2 = mvnrnd(mu2(3:4), cov2, 1)';

conf1 = [conf1; vel1];
conf2 = [conf2; vel2];

pos_interps = [];

% sample from the two ends
for i=1:n_interp
    tau = i*delta_t / n_interp;
    Psi = calQ(Qc, tau) * calPhi(Qc, delta_t - tau)' * invQ(Qc, delta_t);
    Lambda = calPhi(Qc, tau) - Psi * calPhi(Qc, delta_t);
    
    mu_intp = calPhi(Qc, tau) * mu1;
    conf_intp = mu_intp + Lambda * (conf1 - mu1) + Psi * (conf2 - mu2);
    pos_interp = conf_intp(1:2);
    vel_interp = conf_intp(3:4);
    pos_interps = [pos_interps, pos_interp];
end

% sort the interpolations according to x position
[sorted_x, idx] = sort(pos_interps(1, 1:end));
sorted_y = pos_interps(2, idx);

pos_intp_sorted = [sorted_x; sorted_y];

% plotting
figure
hold on
scatter(conf1(1), conf1(2))
scatter(conf2(1), conf2(2))
line(pos_interps(1, 1:end), pos_interps(2, 1:end))
% for i = 1:n_interp-1
%     v1 = pos_intp_sorted(1:2, i)
%     v2 = pos_intp_sorted(1:2, i+1)
%     plot(v1, v2);
%     scatter(v2);
%     line(pos_intp_sorted(1:2, i)', pos_intp_sorted(1:2, i+1)')
% end
