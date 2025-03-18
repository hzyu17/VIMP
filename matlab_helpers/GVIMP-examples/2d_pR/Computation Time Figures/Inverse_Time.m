clear; clc; close all;

x0 = 500;
y0 = 500;
width = 1600;
height = 900;

figure;
set(gca, 'FontName', 'Sans Serif', 'FontSize', 28)

set(gcf, 'Units', 'inches', 'Position', [x0/100 y0/100 width/100 height/100]);
set(gcf, 'PaperUnits', 'inches', 'PaperSize', [width/100, height/100]);
set(gcf, 'PaperPositionMode', 'auto');

hold on;

dimension = [200, 500, 1000, 2000, 3000, 4000, 5000];

gbp_avg_2dpR = [3, 11, 29.8, 86.18, 199.4, 325.32, 458.28];
gbp_min_2dpR = [3, 11, 29, 85, 195, 321, 454];
gbp_max_2dpR = [3, 11, 34, 89, 206, 330, 461];
inverse_avg_2dpR = [3, 15.02, 61.2, 226.14, 499.62, 884.84, 1337.38];
inverse_min_2dpR = [3, 15, 59, 221, 481, 881, 1324];
inverse_max_2dpR = [3, 16, 71, 237, 525, 899, 1370];

gbp_avg_3dpR = [4, 13, 34.0333, 97.1, 204.433, 338.367, 513.033];
gbp_min_3dpR = [4, 13, 34, 97, 203, 337, 512];
gbp_max_3dpR = [4, 13, 35, 98, 207, 340, 515];
inverse_avg_3dpR = [5, 26.5333, 103.4, 399.967, 844.6, 1576, 2432.77];
inverse_min_3dpR = [5, 26, 103, 398, 844, 1573, 2425];
inverse_max_3dpR = [5, 27, 104, 406, 847, 1580, 2440];

plot(dimension, gbp_avg_2dpR, '-o', 'MarkerSize', 16, 'linewidth', 2.5, 'Color', 'g', 'MarkerFaceColor', '[0.5, 1, 0.5]','MarkerEdgeColor', '[0, 0.4, 0]','DisplayName', '2dpR GBP Time Mean');
% fill([dimension, fliplr(dimension)], [gbp_min_2dpR, fliplr(gbp_max_2dpR)], 'g', 'FaceAlpha', 0.15, 'HandleVisibility', 'off');

plot(dimension, inverse_avg_2dpR, '-o', 'MarkerSize', 16, 'linewidth', 2.5, 'Color', 'r', 'MarkerFaceColor', '[1, 0.75, 0.8]', 'MarkerEdgeColor', '[0.4, 0, 0]','DisplayName', '2dpR Inverse Time Mean');
% fill([dimension, fliplr(dimension)], [inverse_min_2dpR, fliplr(inverse_max_2dpR)], 'r', 'FaceAlpha', 0.15, 'HandleVisibility', 'off');

plot(dimension, gbp_avg_3dpR, '--x', 'MarkerSize', 24, 'linewidth', 2.5, 'Color', 'g', 'MarkerEdgeColor', '[0, 0.4, 0]', 'DisplayName', '3dpR GBP Time Mean');
% fill([dimension, fliplr(dimension)], [gbp_min_3dpR, fliplr(gbp_max_3dpR)], 'g', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

plot(dimension, inverse_avg_3dpR, '--x', 'MarkerSize', 24, 'linewidth', 2.5, 'Color', 'r', 'MarkerEdgeColor', '[0.4, 0, 0]', 'DisplayName', '3dpR Inverse Time Mean');
% fill([dimension, fliplr(dimension)], [inverse_min_3dpR, fliplr(inverse_max_3dpR)], 'r', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');



xlabel("Dimension of Precision Matrix")
ylabel("Computation Time (ms)")
legend('show', 'Location', 'northwest');
grid minor

exportgraphics(gcf, 'Inverse_Comparison.pdf', 'ContentType', 'vector', 'BackgroundColor', 'none');