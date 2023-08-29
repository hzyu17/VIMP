function output = plot_3d_result(zk, Sk)
% addpath('../error_ellipse')
output = 1;
nx = 6;

nt = size(zk, 2);
zk_pos = zk(1:3, :);
if size(Sk, 1) ~= 3 && size(Sk, 1) ~= 6
    Sk = reshape(Sk, nx,nx,nt);
end
for i=1:nt
    error_ellipse(Sk(1:3,1:3,i), zk_pos(1:3, i));
    scatter3(zk_pos(1, i), zk_pos(2, i), zk_pos(3, i), 20, 'k', 'fill');
end

end