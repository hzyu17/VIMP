function [roots, W] = getWeight(p)
% Get the sigmapoints and weights for the pth-order Hermite polynomial
c = [1:p-1]';
b = 0;
a = ones(p-1,1);

L = diag(a, 1) + diag(c./a, -1);
roots = eig(L);

% verify the roots
syms x
H0 = 1;
H1 = x;
H2 = x^2-1;
for i=3:p
    H0 = H1;
    H1 = H2;
    H2 = x*H1 - (i-1)*H0;
end
double(subs(H2, roots));

% % plotting
% figure 
% title('H2')
% grid minor
% hold on
% fplot(H2)
% plot(roots, zeros(size(roots)), 'rx')

% Compute the weights
W = factorial(p) / p^2 ./ double(subs(H1, roots)).^2;
end