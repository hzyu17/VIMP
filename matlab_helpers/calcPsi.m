function Psi = calcPsi(Qc, tau, delta_t)
    % Compute Psi for interpolation
    Psi = calcQ(Qc, tau) * (calcPhi(Qc, delta_t - tau)') * invQ(Qc, delta_t);
end