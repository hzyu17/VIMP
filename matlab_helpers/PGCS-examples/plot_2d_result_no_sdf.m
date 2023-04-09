function output = plot_2d_result_no_sdf(zk, Sk)
output = 1;

cell_size = 0.1;
origin_x = -20;
origin_y = -10;

nt = size(zk, 2);
zk_pos = zk(1:2, :);
if size(Sk, 1) ~= 2 && size(Sk, 1) ~= 4
    Sk = reshape(Sk, 4,4,nt);
end
for i=1:nt
    scatter(zk_pos(1, i), zk_pos(2, i), 20, 'k', 'fill');
    error_ellipse(Sk(1:2,1:2,i), zk_pos(1:2, i));
end

end