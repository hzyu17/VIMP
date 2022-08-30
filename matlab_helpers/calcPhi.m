function Phi = calPhi(Qc, dt)
    dim_conf = size(Qc, 1);
    Phi = [eye(dim_conf), dt.*eye(dim_conf); 
                zeros(dim_conf, dim_conf), eye(dim_conf)];
end