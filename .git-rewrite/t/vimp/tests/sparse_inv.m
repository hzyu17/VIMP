function inv_sp = sparse_inv(A)
% Sparse inversion using LDLT
%  Author: Hongzhe Yu
%% inverser considering the sparsity pattern
K = size(A, 1);
% ---------------- LDL factorization ----------------
[L,D] = ldl_golub(A);

Lsp = sparse(L);
[row, col, ~] = find(Lsp);
nnz = size(row, 1);

% --------------- main algorithm -----------------
inv_sp = zeros(K, K);
for index=nnz:-1:1
    j = row(index);
    k = col(index);
    cur_val = 0;
    if (j==k)
        cur_val = 1/D(j,j);
    end
    start_index = index;
    
    for s_index=index:-1:1
        if (col(s_index) < k)
            start_index = s_index+1;
            break;
        end
        if s_index == 1
            start_index = 2;
        end
    end
    for l_indx=start_index : nnz
        l = row(l_indx);
        if (col(l_indx)>k)
            break;
        end
        % ------------ inv_sp(j,l) * L(l,k) ------------
        if(l>j)
            cur_val = cur_val - inv_sp(l,j)*L(l,k);
        else
            cur_val = cur_val - inv_sp(j,l)*L(l,k);
        end
    end
    inv_sp(j,k) = cur_val;
end

inv_sp = inv_sp + tril(inv_sp, -1)';

end