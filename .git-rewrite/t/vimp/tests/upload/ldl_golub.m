% LDL factorization of a symmetrical matrix A.
% 
% [L, D] = LDL_GOLUB(A) returns lower triangular matrix L and diagonal
% matrix D for symmetric matrix A. It should hold that:
%   L*D*L' = A.
% 
% The implementation is a direct rewrite from the book.
% No attempts were made to optimize the function.
% 
% Example:
%     n = 4;                    % size of the generated matrix
%     x = randi(5,n,n);         % square matrix
%     A = x*x';                 % symmetrical matrix
%     [L, D] = ldl_golub(A);
%     obtained = L*D*L';
%     assert(all(all(abs(A - obtained) < eps(200))))

% From Golub 1996:
%   Matrix Computations,
%   algorithm 4.1.2 (in revision from 2013, it is algorithm 4.1.1)

function [L, D] = ldl_golub(A)
n = length(A);
v = zeros(n,1);

for j = 1:n
    for i = 1:j - 1
        v(i) = A(j, i)*A(i, i);
    end
    A(j, j) = A(j, j) - A(j, 1:j - 1) * v(1:j - 1);
    A(j + 1:n, j) = (A(j + 1:n,j) - A(j + 1:n, 1:j - 1) * v(1:j - 1))/A(j, j);
end

L = tril(A,-1)+eye(n);
D = diag(diag(A));
