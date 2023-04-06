function output = plot_3d_result(zk, Sk)
output = 1;
nx = 6;

nt = size(zk, 2);
zk_pos = zk(1:3, :);
Sk = reshape(Sk, nx,nx,nt);
for i=1:nt
    scatter3(zk_pos(1, i), zk_pos(2, i), zk_pos(3, i), 20, 'k', 'fill');
    error_ellipse(Sk(1:3,1:3,i), zk_pos(1:3, i));
end

end