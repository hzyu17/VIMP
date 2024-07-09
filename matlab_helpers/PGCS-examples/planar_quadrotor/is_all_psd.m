function res = is_all_psd(mat3d)
% IS_ALL_PSD: Return if all the matrix in a 3d matrix are all psd.
[~, ~, nt] = size(mat3d);
res = true;
for i=1:nt
    if min(eigs(mat3d(:,:,i)) < 0)
        res = false;
    end
end
res = true;
end

