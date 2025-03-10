clear; clc; close all;

x0 = 500;
y0 = 500;
width = 1600
height = 900;

set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');

figure;
set(gca, 'FontName', 'Sans Serif', 'FontSize', 28)
% set(gcf, 'position',[x0,y0,width,height])

set(gcf, 'Units', 'inches', 'Position', [x0/100, y0/100, width/100, height/100]);
set(gcf, 'PaperUnits', 'inches', 'PaperSize', [width/100, height/100]);
set(gcf, 'PaperPositionMode', 'auto');

hold on;

%% 3dpR GH Degree = 4
n_states = [25, 50, 100, 200, 300, 500, 750, 1000];

% GPU times (in ms)
GPU_avg = [0.8, 1.1, 2.4, 4.45, 6.65, 14.55, 24.2, 41.15];
GPU_min = [0, 1, 1, 4, 6, 12, 23, 37];
GPU_max = [2, 2, 4, 6, 8, 17, 27, 48];

% CPU times (in ms)
CPU_avg = [2.2, 2.95, 7, 17.35, 29.35, 95.6, 179.8, 291.85];
CPU_min = [1, 2, 6, 15, 26, 92, 173, 282];
CPU_max = [8, 4, 11, 21, 32, 101, 190, 303];

ratio = (CPU_avg-GPU_avg)./CPU_avg

gpu1 = plot(n_states, GPU_avg, '-o', 'MarkerSize', 14, 'LineWidth', 2.5, 'Color', 'g', ...
    'MarkerFaceColor', [0.5, 1, 0.5], 'MarkerEdgeColor', [0, 0.35, 0], 'DisplayName', 'GPU ($k_q = 4$)');
% fill([n_states, fliplr(n_states)], [GPU_min, fliplr(GPU_max)], 'g', 'FaceAlpha', 0.15, 'HandleVisibility', 'off');

cpu1 = plot(n_states, CPU_avg, '-o', 'MarkerSize', 14, 'LineWidth', 2.5, 'Color', 'r', ...
    'MarkerFaceColor', [1, 0.75, 0.8],  'MarkerEdgeColor', [0.35, 0, 0], 'DisplayName', 'CPU ($k_q = 4$)');
% fill([n_states, fliplr(n_states)], [CPU_min, fliplr(CPU_max)], 'r', 'FaceAlpha', 0.15, 'HandleVisibility', 'off');

%% 3dpR GH Degree = 10
n_states = [25, 50, 100, 200, 300, 500, 750, 1000];

% GPU times (in ms)
GPU_avg = [2.4, 2.8, 4.7, 8.9, 15.95, 25.7, 37.6, 56.1];
GPU_min = [2, 2, 4, 7, 13, 24, 35, 53];
GPU_max = [4, 4, 7, 12, 19, 28, 40, 59];

% CPU times (in ms)
CPU_avg = [17.4, 33.85, 69.8, 140.9, 209.25, 384.7, 598.65, 852];
CPU_min = [15, 31, 65, 134, 196, 373, 576, 833];
CPU_max = [20, 38, 75, 162, 221, 413, 631, 866];

ratio = (CPU_avg-GPU_avg)./CPU_avg

gpu2 = plot(n_states, GPU_avg, '--x', 'MarkerSize', 16, 'LineWidth', 2.5, 'Color', 'g', ...
    'MarkerEdgeColor', [0, 0.35, 0], 'DisplayName', 'GPU ($k_q = 10$)');
% fill([n_states, fliplr(n_states)], [GPU_min, fliplr(GPU_max)], 'g', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

cpu2 = plot(n_states, CPU_avg, '--x', 'MarkerSize', 16, 'LineWidth', 2.5, 'Color', 'r', ...
    'MarkerEdgeColor', [0.35, 0, 0], 'DisplayName', 'CPU ($k_q = 10$)');
% fill([n_states, fliplr(n_states)], [CPU_min, fliplr(CPU_max)], 'r', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

%% 3dpR GH Degree = 16
n_states = [25, 50, 100, 200, 300, 500, 750, 1000];

% GPU times (in ms)
GPU_avg = [12.9, 18.3, 24.4, 40.15, 62.65, 91.15, 130.85, 177.8];
GPU_min = [11, 16, 21, 37, 57, 84, 123, 172];
GPU_max = [14, 21, 29, 47, 80, 100, 142, 185];

% CPU times (in ms)
CPU_avg = [142.15, 264.15, 542.75, 1022.55, 1605.4, 2620.85, 3891.75, 5246.3];
CPU_min = [131, 251, 527, 995, 1562, 2560, 3809, 5127];
CPU_max = [156, 287, 561, 1046, 1680, 2667, 3972, 5441];

ratio = (CPU_avg-GPU_avg)./CPU_avg

gpu3 = plot(n_states, GPU_avg, '-.d', 'MarkerSize', 14, 'LineWidth', 2.5, 'Color', 'g', ...
    'MarkerFaceColor', [0.5, 1, 0.5], 'MarkerEdgeColor', [0, 0.35, 0], 'DisplayName', 'GPU ($k_q = 16$)');
% fill([n_states, fliplr(n_states)], [GPU_min, fliplr(GPU_max)], 'g', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

cpu3 = plot(n_states, CPU_avg, '-.d', 'MarkerSize', 14, 'LineWidth', 2.5, 'Color', 'r', ...
    'MarkerFaceColor', [1, 0.75, 0.8], 'MarkerEdgeColor', [0.35, 0, 0], 'DisplayName', 'CPU ($k_q = 16$)');
% fill([n_states, fliplr(n_states)], [CPU_min, fliplr(CPU_max)], 'r', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

xlabel("Time Discretizations")
ylabel("Cost Evaluation Time (ms)")
legend([gpu1, gpu2, gpu3, cpu1, cpu2, cpu3], 'Location', 'northwest');
% legend([gpu1, gpu2, gpu3, cpu1, cpu2], 'Location', 'northwest');
grid minor
% ylim([0, 5500])

% set(gcf, 'Toolbar', 'none')
% exportgraphics(gcf, '3dpR_1_updated.pdf', 'ContentType', 'vector', 'BackgroundColor', 'none');
