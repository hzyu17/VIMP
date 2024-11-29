clear; clc; close all;

x0 = 500;
y0 = 500;
width = 1200;
height = 900;

figure;
hold on;
set(gca, 'FontName', 'Sans Serif', 'FontSize', 28)
set(gcf, 'Units', 'inches', 'Position', [x0/100 y0/100 width/100 height/100]);
set(gcf, 'PaperUnits', 'inches', 'PaperSize', [width/100, height/100]);
set(gcf, 'PaperPositionMode', 'auto');

% GH Degree = 4
n_states = [25, 50, 100, 200, 300, 400, 500, 750, 1000];

% GPU times
GPU_avg_4 = [15.2333, 23.1333, 43.8333, 88.2667, 145.867, 207.767, 273.967, 499.2, 777.533] * 35 / 1000;
GPU_min_4 = [11, 20, 39, 83, 138, 195, 266, 487, 755] * 35 / 1000;
GPU_max_4 = [20, 26, 52, 96, 160, 223, 288, 516, 809] * 35 / 1000;

CPU_avg_4 = [22.6667, 43.9, 80.2, 181.267, 291.8, 415.067, 529.1, 944.567, 1404.03] * 35 / 1000;
CPU_min_4 = [19, 39, 74, 167, 274, 401, 504, 921, 1381] * 35 / 1000;
CPU_max_4 = [28, 69, 90, 213, 324, 430, 547, 975, 1461] * 35 / 1000;

plot(n_states, GPU_avg_4, '-o', 'MarkerSize', 16, 'LineWidth', 2.5, 'Color', 'g', 'MarkerFaceColor', [0.5, 1, 0.5], 'MarkerEdgeColor', [0, 0.4, 0], 'DisplayName', 'GPU Mean GH degree = 4');
fill([n_states, fliplr(n_states)], [GPU_min_4, fliplr(GPU_max_4)], 'g', 'FaceAlpha', 0.15, 'HandleVisibility', 'off');

plot(n_states, CPU_avg_4, '-o', 'MarkerSize', 16, 'LineWidth', 2.5, 'Color', 'r', 'MarkerFaceColor', [1, 0.75, 0.8], 'MarkerEdgeColor', [0.4, 0, 0], 'DisplayName', 'CPU Mean GH degree = 4');
fill([n_states, fliplr(n_states)], [CPU_min_4, fliplr(CPU_max_4)], 'r', 'FaceAlpha', 0.15, 'HandleVisibility', 'off');

% GH Degree = 10
% GPU times
GPU_avg_10 = [20.4667, 35.3333, 63.6667, 134.733, 209.267, 288.167, 373.8, 661.4, 997.4] * 35 / 1000;
GPU_min_10 = [18, 33, 58, 123, 193, 273, 361, 639, 958] * 35 / 1000;
GPU_max_10 = [26, 40, 75, 150, 225, 306, 393, 724, 1081] * 35 / 1000;

CPU_avg_10 = [60.8, 115.567, 214.267, 445.2, 664.967, 875.23, 1123.33, 1783.13, 2540.33] * 35 / 1000;
CPU_min_10 = [53, 99, 200, 418, 623, 822, 1079, 1708, 2457] * 35 / 1000;
CPU_max_10 = [68, 151, 236, 507, 753, 973, 1223, 1907, 2652] * 35 / 1000;

plot(n_states, GPU_avg_10, '--x', 'MarkerSize', 24, 'LineWidth', 2.5, 'Color', 'g', 'MarkerFaceColor', [0.5, 1, 0.5], 'MarkerEdgeColor', [0, 0.4, 0], 'DisplayName', 'GPU Mean GH degree = 10');
fill([n_states, fliplr(n_states)], [GPU_min_10, fliplr(GPU_max_10)], 'g', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

plot(n_states, CPU_avg_10, '--x', 'MarkerSize', 24, 'LineWidth', 2.5, 'Color', 'r', 'MarkerFaceColor', [1, 0.75, 0.8], 'MarkerEdgeColor', [0.4, 0, 0], 'DisplayName', 'CPU Mean GH degree = 10');
fill([n_states, fliplr(n_states)], [CPU_min_10, fliplr(CPU_max_10)], 'r', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

ratio = (CPU_avg_10-GPU_avg_10)./CPU_avg_10

xlabel("Time Discretizations")
ylabel("Optimization Time Consumption (s)")
legend('show', 'Location', 'northwest');
grid minor

exportgraphics(gcf, '2dpR_total_1.pdf', 'ContentType', 'vector', 'BackgroundColor', 'none');