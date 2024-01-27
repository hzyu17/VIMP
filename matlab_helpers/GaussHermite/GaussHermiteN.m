function Integral_GHs = GaussHermiteN(n, gx, p, m, P)
% Gaussian-Hermite approximation of the integration of gx w.r.t. a Gaussian
% distribution
% Input:
%           n: dimension of variables: n=2 or 3
%           gx: integrand, which is supposed to be a symbolic expression in
%           variable sym('x', [n 1])
%           p: order of the Hermite polynomial
%           m: mean
%           P: covariance

x = sym('x', [n 1]);
sig = sqrtm(P);
gaussian_x = det(2*pi*P)^(-1/2) * expm(-transpose(x-m)*inv(P)*(x-m)/2);

integral_trues = [];
Integral_GHs = [];
for i_size = 1:size(gx, 1)
    func_hd = matlabFunction(gx(i_size)*gaussian_x);

    % weights for Gaussian Hermite quadratures
    [ksi, W] = getWeight(p);
    % ksi = repmat(ksi', [n 1]); % dimension: (n, p)
    % W = repmat(W', [n 1]); % dimension: (n, p)
    
    Integral_GH = 0;
    if n == 2
        integral_trues = [integral_trues; integral2(func_hd, -inf, inf, -inf, inf)];
        for i=1:p
            for j = 1:p
                pts = sig*[ksi(i); ksi(j)] + m;
                integrand = double(subs(gx, x, pts));
                integrand = integrand(i_size);
                i_W = W(i) * W(j);
                Integral_GH = Integral_GH + integrand * i_W;
            end
        end
    elseif n ==3
        integral_trues =  [integral_trues; integral3(func_hd, -inf, inf, -inf, inf, -inf, inf)];
        for i=1:p
            for j = 1:p
                for k = 1:p
                pts = sig*[ksi(i); ksi(j); ksi(k)] + m;
                integrand = double(subs(gx, x, pts));
                integrand = integrand(i_size);
                i_W = W(i) * W(j) * W(k);
                Integral_GH = Integral_GH + integrand * i_W;
                end
            end
        end
    end
    Integral_GHs = [Integral_GHs; Integral_GH];
end
integral_trues;
end