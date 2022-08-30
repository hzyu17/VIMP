function Lambda = calcLambda(Qc, tau, delta_t)
    % Compute Lambda for interpolation
    Lambda = calcPhi(Qc, tau) - calcQ(Qc, tau) * calcPhi(Qc, delta_t - tau)' * invQ(Qc, delta_t) * calcPhi(Qc, delta_t);
end