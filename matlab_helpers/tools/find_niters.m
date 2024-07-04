function niters = find_niters(means_3d)
% FIND_NITERS Find the actual number of iterations of one experiment
% Input is the 3d matrix which stores the means of the results.

[nstate, nt] = size(means_3d);
for i = 1:nt
    sum(means_3d(:,i))
    if sum(means_3d(:,i)) == 0
        niters = i-1;
        return;
    end
end
niters = nt;
end

