% LDL factorization of a matrix of the form ZZ' + Λ.
% 
% [L, D] = LDL_SMOLA(Z, A) returns lower triangular matrix L and diagonal
% matrix D for rectangular matrix Z and regularization vector A, which
% represents a diagonal matrix. It should hold that:
%   L*D*L' = Z*Z' + diag(A).
% 
% The implementation is a direct rewrite from the article.
% No attempts were made to optimize the function.
% 
% Example:
%     m = 4;                    % count of columns in Smola's notation
%     n = 5;                    % count of rows
%     Z = randi(5,n,m);         % rectangular input data 
%     a = ones(n,1);            % regularization
%     [L, D] = ldl_smola(Z, a);
%     expected = Z*Z'+diag(a);
%     obtained = L*D*L';
%     assert(all(all(abs(expected - obtained) < eps(200))))

% From Smola & Vishwanathan:
%   LDL Factorization for Rank-k Modifications of Diagonal Matrices,
%   Algorithm 2.1: Triangular Factorization.

function [L, D] = ldl_smola(Z, a)
[n, m] = size(Z);   % Smola's notation from the article
M = eye(m);         % auxiliary matrix
B = zeros(n,m);     
d = zeros(n,1);     % diagonal vector

for i=1:n
    t = M*Z(i,:)';
    d(i) = a(i) + Z(i,:)*t;
    if d(i)>0
        B(i,:) = 1/d(i)*t;
        M = M-(1/d(i)) * (t * t');
    end
end

L = tril(Z*B', -1) + eye(n);
D = diag(d);
