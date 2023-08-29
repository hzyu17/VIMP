hinge_mesh = csvread("../../vimp/tests/data/mesh_hinge_loss.csv");
% hinge_mesh_jacobian = csvread("../vimp/tests/data/mesh_hinge_jacobians.csv");
% mesh hinge jacobian needs to be loaded by hands
x_mesh = csvread("../../vimp/tests/data/sdf_grid_x.csv");
y_mesh = csvread("../../vimp/tests/data/sdf_grid_y.csv");

xlen = 400;
ylen = 300;

X_mesh = repmat(x_mesh', 1, ylen);
Y_mesh = repmat(y_mesh, xlen, 1);

figure
title('Hinge Loss')
surf(X_mesh, Y_mesh, hinge_mesh);

figure
grid minor
hold on
title("Hinge Loss Jacobian")
Mesh_jacobian = zeros(xlen, ylen, 2);
for i=1:xlen
    for j=1:ylen
        Mesh_jacobian(i, j, :) = meshhingejacobians(:, i*j);
    end
end
quiver(X_mesh, Y_mesh, Mesh_jacobian(:,:,1), Mesh_jacobian(:,:,2));

%%
for i = 1:xlen
    for j = 1:ylen
        hingeloss_ij = hinge_mesh(i, j);
        hingeloss_jacobian_ij = hinge_mesh_jacobian(i,j);
    end
end