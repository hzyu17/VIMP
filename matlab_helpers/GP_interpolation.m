clear all
clc

%% ====== GP interpolation ======
w_Qc = 0.8;
dim_conf = 2;
Qc = eye(dim_conf) .* w_Qc;
ttl_time = 2.0;
n_states = 15;
dt = ttl_time / n_states;
n_interp = 10;
tau = dt / n_interp;

x1 = [0; 0; 0.1; 0.4];
x2 = [1.0; 2.0; 0.0; 0.0];

figure
hold on
for i = 1:n_interp
    i_tau = tau*i;
    Lambda = calcLambda(Qc, i_tau, dt);
    Psi = calcPsi(Qc, i_tau, dt);
    interp = Lambda(1:dim_conf, 1:2*dim_conf) * x1 + Psi(1:dim_conf, 1:2*dim_conf) * x2
    scatter(interp(1), interp(2));
end