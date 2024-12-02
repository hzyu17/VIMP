clear; clc; close all;

x0 = 500;
y0 = 500;
width = 1200;
height = 900;

figure;
set(gca, 'FontName', 'Sans Serif', 'FontSize', 30)
% set(gcf, 'position',[x0,y0,width,height])

set(gcf, 'Units', 'inches', 'Position', [x0/100 y0/100 width/100 height/100]);
set(gcf, 'PaperUnits', 'inches', 'PaperSize', [width/100, height/100]);
set(gcf, 'PaperPositionMode', 'auto');

hold on;

%% GH Degree = 4
n_states = [10, 25, 50, 100, 200, 300, 500];

GPU_avg = [3.7, 6.46667, 11.5333, 24.7667, 61.1667, 107.267, 245.167];
GPU_min = [3, 6, 11, 23, 57, 100, 237];
GPU_max = [7, 11, 16, 32, 67, 119, 264];

CPU_avg = [10.8333, 26.4, 50.5333, 105.45, 221.533, 358.867, 725.8];
CPU_min = [8, 22, 46, 97, 193, 312, 670];
CPU_max = [17, 31, 59, 113, 237, 381, 777];

plot(n_states, GPU_avg, '-o', 'MarkerSize', 16, 'linewidth', 2.5, 'Color', 'g', 'MarkerFaceColor', '[0.5, 1, 0.5]','MarkerEdgeColor', '[0, 0.4, 0]','DisplayName', 'GPU Mean GH degree = 4');
fill([n_states, fliplr(n_states)], [GPU_min, fliplr(GPU_max)], 'g', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

plot(n_states, CPU_avg, '-o', 'MarkerSize', 16, 'linewidth', 2.5, 'Color', 'r', 'MarkerFaceColor', '[1, 0.75, 0.8]', 'MarkerEdgeColor', '[0.4, 0, 0]','DisplayName', 'CPU Mean GH degree = 4');
fill([n_states, fliplr(n_states)], [CPU_min, fliplr(CPU_max)], 'r', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');


%% GH Degree = 10
GPU_avg = [23.6667, 61.1667, 238.3, 488.5, 843.333, 1172.67, 1451.55];
GPU_min = [20, 52, 232, 471, 813, 1139, 1426];
GPU_max = [31, 69, 244, 505, 942, 1231, 1481];

% CPU 时间向量
CPU_avg = [165.467, 457.433, 1627.77, 3268.6, 4814.27, 6582.1, 8199.1];
CPU_min = [136, 401, 1568, 3191, 4720, 6504, 8040];
CPU_max = [179, 560, 1694, 3379, 4950, 6669, 8318];

ratio = (CPU_avg-GPU_avg)./CPU_avg;

plot(n_states, GPU_avg, '--x', 'MarkerSize', 24, 'linewidth', 2.5, 'Color', 'g', 'MarkerEdgeColor', '[0, 0.4, 0]','DisplayName', 'GPU Mean GH degree = 10');
fill([n_states, fliplr(n_states)], [GPU_min, fliplr(GPU_max)], 'g', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

plot(n_states, CPU_avg, '--x', 'MarkerSize', 24, 'linewidth', 2.5, 'Color', 'r', 'MarkerEdgeColor', '[0.4, 0, 0]','DisplayName', 'CPU Mean GH degree = 10');
fill([n_states, fliplr(n_states)], [CPU_min, fliplr(CPU_max)], 'r', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');


%% GH Degree = 16
GPU_avg = [198.65, 543.7, 1133.75, 2332, 4724.25, 7049.1, 11791.1];
GPU_min = [179, 523, 1105, 2279, 4648, 6946, 11605];
GPU_max = [208, 567, 1166, 2489, 4820, 7207, 11901];

CPU_avg = [1349.95, 3635.9, 7051.45, 13865.8, 27812.6, 41193.7, 68592.9];
CPU_min = [1291, 3545, 6909, 13679, 27387, 40788, 67977];
CPU_max = [1480, 3715, 7274, 14117, 28166, 41666, 69467];

% plot(n_states, GPU_avg, 'g--d', 'MarkerSize', 10, 'linewidth', 2.5, 'DisplayName', 'GPU Mean GH degree = 16');
% fill([n_states, fliplr(n_states)], [GPU_min, fliplr(GPU_max)], 'g', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

% plot(n_states, CPU_avg, 'r--d', 'MarkerSize', 10, 'linewidth', 2.5, 'DisplayName', 'CPU Mean GH degree = 16');
% fill([n_states, fliplr(n_states)], [CPU_min, fliplr(CPU_max)], 'r', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

ratio = (CPU_avg-GPU_avg)./CPU_avg;


xlabel("Time Discretizations")
ylabel("Computation Time (ms)")
legend('show', 'Location', 'northwest');
grid minor

exportgraphics(gcf, '3dpR_1.pdf', 'ContentType', 'vector', 'BackgroundColor', 'none');
