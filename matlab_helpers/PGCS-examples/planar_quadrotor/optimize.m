function [K,d,As,B,as,zk,Sk] = optimize(nt, As, B, pinvBBT, as, Q, epsilon, m0, Sig0, m1, Sig1, eta, sig, stop_error)
%optimize: Function that opoerates the main iteration in the algorithm.

As_prev = As;
as_prev = as;
err = 1;
iter = 0;

% zk = zeros(nx, nt);
% for i=1:nx
% zk(i,:) = linspace(m0(i), m1(i), nt);
% end
% Sk = Sig0(:)*ones(1, nt);
% Sk = reshape(Sk, [nx,nx,nt]);
while (err>stop_error)
    iter = iter + 1;

    [Sk, zk]       = Szk(As,B,as,epsilon,m0,Sig0,sig);
    [hAk, hak, nTr] = linearAa_pquad(Sk,zk,As);
    
    Aprior   = (1/(1+eta)).*As+(eta/(1+eta)).*hAk;
    aprior   = (1/(1+eta)).*as+(eta/(1+eta)).*hak;

    [Qk, rk] = QrV(As,as,hAk,hak,nTr,eta,B, pinvBBT, Q,zk);
    [K,d] = linearCov(Aprior,aprior,B,epsilon,Qk,rk,m0,Sig0,m1,Sig1,sig);
    
    Qcpp = csvread("/home/hzyu/git/VIMP/vimp/Qt.csv");
    rcpp = csvread("/home/hzyu/git/VIMP/vimp/rt.csv");
    Kcpp = csvread("/home/hzyu/git/VIMP/vimp/Kt.csv");
    dcpp = csvread("/home/hzyu/git/VIMP/vimp/dt.csv");
    
    Qk_reshaped = reshape(Qk, [36, 500]);
    K_reshaped = reshape(K, [12, 500]);
    
    K_diff = Kcpp - K_reshaped;
    Q_diff = Qk_reshaped - Qcpp;
    
    err_norm_K = norm(K_diff)
    err_norm_Q = norm(Q_diff)
    
    
    for i = 1:nt
        As(:,:,i) = Aprior(:,:,i)+B(:,:,i)*K(:,:,i);
        as(:,i) = aprior(:,i)+B(:,:,i)*d(:,i);
    end
    
    err = norm(As_prev(:)-As(:), 'fro')/norm(As(:), 'fro')/nt + norm(as_prev(:)-as(:), 'fro')/norm(as(:), 'fro')/nt
    As_prev = As;
    as_prev = as;
end
end