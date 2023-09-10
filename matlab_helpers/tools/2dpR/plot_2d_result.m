function output = plot_2d_result(sdfmap, zk, Sk, step ,args)
output = 1;

cell_size = 0.1;
origin_x = -20;
origin_y = -10;

plotEvidenceMap2D_1(sdfmap, origin_x, origin_y, cell_size);

nt = size(zk, 2);
zk_pos = zk(1:2, :);
if size(Sk, 1) ~= 2 && size(Sk, 1) ~= 4
    Sk = reshape(Sk, 4,4,nt);
end

plot(zk_pos(1,:), zk_pos(2,:), '-', 'Color',[0.301 0.745 0.933], 'LineWidth', 0.6);
for i=1:step:nt
    % scatter(zk_pos(1, i), zk_pos(2, i), 20, 'k', 'fill');
    error_ellipse(Sk(1:2,1:2,i), zk_pos(1:2, i),'style',args);
end
axis off
end