%% Recovering the one dimensional case
% Author: Hongzhe Yu
% Date: 07/26/2022

%% algorithm iterations
clear all
clc
syms x real

% problem related
mu_p = 20;
sig_p_sq = 9;
f = 400;
b = 0.1;
sig_r_sq = 0.09;
y = f*b/mu_p - 0.8;

niters = 5;
ss = 0.75;
B = 1;
p = 20;
mu = mu_p;
sig_sq = sig_p_sq;
prec = 1/sig_sq;

% *****************************  phi(x) ************************** (85)
T = 1;
phi = T * ((x-mu_p).^2./sig_p_sq./2 + (y - f*b./x).^2./sig_r_sq./2);

mus = [];
precs = [];
costs = [];

for i_iter = 1:niters
    
    % *********************** current cost ************************ (85)
    disp(['===== iteration', num2str(i_iter-1), ' ====='])
    mu
    sig_sq = 1/prec;
    prec
    cost = GaussHermitOneDim(phi, p, mu, 1/prec) + log(prec)/2
    disp("log(prec)")
    log(prec)
    mus = [mus, mu];
    precs = [precs, prec];
    costs = [costs, cost];
    
    % ************************** (52) *********************************
    xmu_phi = (x-mu).*phi;
    xmumuT_phi = (x-mu)*(x-mu).*phi;
       
    % ************************* calculate (10) **************************
%     disp("--- Vdmu ---")
%     GaussHermitOneDim(xmu_phi, p, mu, 1/prec)
%     disp("--- _precision * Vdmu ---")
%     prec * GaussHermitOneDim(xmu_phi, p, mu, 1/prec)

%     disp("--- Vddmu = E_{q_k}{(x_k - mu_k)(x_k - mu_k)^T * phi(x_k)} ---")
%     GaussHermitOneDim(xmumuT_phi, p, mu, 1/prec) 

    Vdmu = prec * GaussHermitOneDim(xmu_phi, p, mu, 1/prec);
%     disp("--- Vddmu ---")
%     prec * GaussHermitOneDim(xmumuT_phi, p, mu, 1/prec) * prec -  prec * GaussHermitOneDim(phi, p, mu, 1/prec)
    Vddmu = prec * GaussHermitOneDim(xmumuT_phi, p, mu, 1/prec) * prec -  prec * GaussHermitOneDim(phi, p, mu, 1/prec);
    
    % ************************ (14) (15) *******************************
    d_prec = Vddmu - prec
    dmu = -Vdmu / Vddmu
    
    % *********************** for backtracking ************************
    new_mu = mu + ss * dmu;
    new_prec = prec + ss * d_prec;
    
    % ********************** backtracking *****************************
    new_cost = GaussHermitOneDim(phi, p, new_mu, 1/new_prec) + log(new_prec)/2;
    cnt = 0;
    while (new_cost > cost)
        % *************************** (80) **********************************
        B = B+1;
        sss = ss^B;
        new_mu = mu + sss * dmu;
        new_prec = prec + sss * d_prec;
        % ------------------- compute new cost -----------------
        new_cost = GaussHermitOneDim(phi, p, new_mu, 1/new_prec) + log(new_prec)/2;

        cnt = cnt + 1;
        if cnt > 500
            disp("Over 500 times shrinking ... ")
            break
        end
    end
    % ********************** update step *****************************
    mu = new_mu;
    prec = new_prec;
    B = 0;
end

%% ************************************** plotting the results ******************************************
disp("---- plotting ----")

% ====== reading the cost map data, generated from cpp =======
cpp_costmap = csvread("../../vimp/data/vimp/1d/costmap.csv");
nmesh = 40;
x_mesh = linspace(18, 25, nmesh);
y_mesh = linspace(0.05,1,nmesh);
[X,Y] = meshgrid(x_mesh, y_mesh);
% cpp_costmap(find(cpp_costmap>10))=0;

% ========================= matlab iterations =========================
subplot(2,2,1)
title('iterations')
hold on
contourf(X,Y,cpp_costmap,40);
grid on
plot(mus, precs, 'c', 'LineWidth', 2.2)
for i_iter = 1:niters
    scatter(mus(i_iter), precs(i_iter), 'r*');
end

xlabel("mu")
ylabel("sig^{-2}")

% ======================= cpp iterations =======================
cpp_means  = csvread("../../vimp/data/vimp/1d/mean.csv");
cpp_covs = csvread("../../vimp/data/vimp/1d/cov.csv");
cpp_costs = csvread("../../vimp/data/vimp/1d/cost.csv");
cpp_precs = 1./cpp_covs;
subplot(2,2,2)
title("iterations cpp")
grid on
hold on
contourf(X,Y,cpp_costmap,40);
plot(cpp_means, cpp_precs,'c', 'LineWidth', 2.2)

for i_iter = 1:niters
    scatter(cpp_means(i_iter), cpp_precs(i_iter), 'r*');
end
xlabel("mu")
ylabel("sig^{-2}")

% ========================== matlab cost values ===========================
subplot(2,2,3)
title("V(q)")
hold on
grid on
for i_iter = 1:niters
    plot(costs, 'LineWidth', 2.2)
end
xlabel("iterations")
ylabel("V(q)")

% =========================== cpp cost values ==========================
subplot(2,2,4)
title("V(q) cpp")
hold on
grid on
for i_iter = 1:niters
    plot(cpp_costs, 'LineWidth', 2.2)
end
xlabel("iterations")
ylabel("V(q)")

