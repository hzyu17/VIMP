function Q = calQ(Qc, dt)
    dim_conf = size(Qc, 1);
    Q = [dt^3.*Qc/3, dt^2.*Qc/2; 
             dt^2.*Qc/2, dt.*Qc];
end