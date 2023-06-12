function res = plot_mat3_elements(mat3d, n,m, i, j)
%PLOT_MAT3_ELEMENTS Plot all the element in function of time of a 3D
%matrix.
nt = size(mat3d, 2);
mat = reshape(mat3d, [n,m, nt]);
vec_ijt = zeros(1, nt);

for i_t=1:nt
    vec_ijt(i_t) = mat(i,j,i_t);
end

figure
grid minor
hold on
plot(linspace(1,nt, nt), vec_ijt, LineWidth=2.0)

res = true;

end

