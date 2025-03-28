clear; clc; close all;

x0 = 500;
y0 = 500;
width = 1600;
height = 900;

figure;
set(gca, 'FontName', 'Sans Serif', 'FontSize', 32)

set(gcf, 'Units', 'inches', 'Position', [x0/100 y0/100 width/100 height/100]);
set(gcf, 'PaperUnits', 'inches', 'PaperSize', [width/100, height/100]);
set(gcf, 'PaperPositionMode', 'auto');

hold on;

dimension = [200, 500, 1000, 2000, 3000, 4000, 5000];

gbp_avg_2dpR = [0, 0, 0, 1, 1, 2, 2.36];
inverse_avg_2dpR = [0, 2, 9.08, 39, 92.82, 165.32, 256];

gbp_avg_3dpR = [0, 0.14, 0.16, 1.2, 2.04, 2.06, 3];
inverse_avg_3dpR = [0.04, 4.3, 18.02, 72.54, 166.34, 297.76, 456.86];

gbp_avg_7dof = [0, 0.06, 1, 2.06, 3.1, 4.2, 6.04];
inverse_avg_7dof = [2, 14, 49.58, 197.44, 439.32, 815.1, 1234.04];

plot(dimension, gbp_avg_2dpR, '-o', 'MarkerSize', 18, 'linewidth', 3, 'Color', '[0, 0.5, 0]', 'MarkerFaceColor', '[0.5, 1, 0.5]','MarkerEdgeColor', '[0, 0.4, 0]','DisplayName', '2D Point Robot GBP');
plot(dimension, inverse_avg_2dpR, '-o', 'MarkerSize', 18, 'linewidth', 3, 'Color', 'r', 'MarkerFaceColor', '[1, 0.75, 0.8]', 'MarkerEdgeColor', '[0.4, 0, 0]','DisplayName', '2D Point Robot Inverse');
plot(dimension, gbp_avg_3dpR, '--x', 'MarkerSize', 26, 'linewidth', 3, 'Color', '[0, 0.5, 0]', 'MarkerEdgeColor', '[0, 0.4, 0]', 'DisplayName', '3D Point Robot GBP');
plot(dimension, inverse_avg_3dpR, '--x', 'MarkerSize', 26, 'linewidth', 3, 'Color', 'r', 'MarkerEdgeColor', '[0.4, 0, 0]', 'DisplayName', '3D Point Robot Inverse');
plot(dimension, gbp_avg_7dof, ':s', 'MarkerSize', 18, 'linewidth', 3, 'Color', '[0, 0.5, 0]', 'MarkerFaceColor', '[0.5, 1, 0.5]','MarkerEdgeColor', '[0, 0.4, 0]', 'DisplayName', '7DoF WAM GBP');
plot(dimension, inverse_avg_7dof, ':s', 'MarkerSize', 18, 'linewidth', 3, 'Color', 'r', 'MarkerFaceColor', '[1, 0.75, 0.8]', 'MarkerEdgeColor', '[0.4, 0, 0]', 'DisplayName', '7DoF WAM Inverse');

xlabel("Dimension of Joint Precision Matrix")
ylabel("Inverse Time (ms)")
legend('show', 'Location', 'northwest');
grid minor

exportgraphics(gcf, 'Inverse_Comparison.pdf', 'ContentType', 'vector', 'BackgroundColor', 'none');