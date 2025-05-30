function output = plot_2d_result(sdfmap, zk, Sk, cell_size, origin_x, origin_y)
output = 1;

plotEvidenceMap2D_1(sdfmap, origin_x, origin_y, cell_size);

nt = size(zk, 2);
zk_pos = zk(1:2, :);
if size(Sk, 1) ~= 2 && size(Sk, 1) ~= 6
    Sk = reshape(Sk, 6, 6, nt);
end
for i=1:nt
    scatter(zk_pos(1, i), zk_pos(2, i), 20, 'k', 'fill');
    error_ellipse(Sk(1:2,1:2,i), zk_pos(1:2, i));
end
% axis off
end