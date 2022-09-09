%% calculate the permutation for sigmapoints
% author: Hongzhe Yu 
% date: 08/01/22
% https://www.geeksforgeeks.org/print-all-permutations-with-repetition-of-characters/

% permute list with replacement. 
% I.E., vec=[1,2,3], k=2 -> {[1,1], [1,2], [1,3], [2,1], [2,2], [2,3], [3,1],[3,2],[3,3]}

function [c_res, res] = permute_with_replace(vec, k, res, index, c_res)
    for i=1:length(vec)
        res(index) = vec(i);
        if index == k
            c_res = {c_res{1:end}, res};
        else
            [c_res, res] = permute_with_replace(vec, k, res, index+1, c_res);
        end
    end
    return
end
