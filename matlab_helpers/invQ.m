function invQ = invQ(Qc, dt)
Qc_inv = inv(Qc);
invQ = [12.0 * dt^(-3) .* Qc_inv, (-6.0) * dt^(-2) .* Qc_inv;
              (-6.0) * dt^(-2) .* Qc_inv, 4.0 * dt^(-1) .* Qc_inv];
end