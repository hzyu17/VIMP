vec_res = []
l = [1,2,3];
k = 3;
res = zeros(1,k);
c_res = {};
[c_res,  res] = permute_with_replace(l, k, res, 1, c_res)