clear; clc; close all;

x0 = 500;
y0 = 500;
width = 1600;
height = 900;

set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');

figure;
set(gca, 'FontName', 'Sans Serif', 'FontSize', 28)
% set(gcf, 'position',[x0,y0,width,height])

set(gcf, 'Units', 'inches', 'Position', [x0/100 y0/100 width/100 height/100]);
set(gcf, 'PaperUnits', 'inches', 'PaperSize', [width/100, height/100]);
set(gcf, 'PaperPositionMode', 'auto');

hold on;

%% WAM GH Degree = 3
n_states = [25, 50, 100, 200, 300, 500, 750];

% GPU times (in ms)
GPU_avg = [2.76667, 5.63333, 12.7, 19.8, 30.0667, 48.1, 78.8];
GPU_min = [2, 4, 9, 16, 23, 40, 74];
GPU_max = [4, 7, 15, 26, 39, 56, 88];

% CPU times (in ms)
CPU_avg = [16.1, 30.3, 57.8, 161.75, 269.35, 547.35, 1002.4];
CPU_min = [14, 28, 52, 152, 248, 522, 970];
CPU_max = [19, 34, 64, 177, 287, 578, 1060];

ratio = (CPU_avg-GPU_avg)./CPU_avg

gpu1 = plot(n_states, GPU_avg, '-o', 'MarkerSize', 14, 'LineWidth', 3, 'Color', 'g', ...
    'MarkerFaceColor', '[0.5, 1, 0.5]', 'MarkerEdgeColor', [0, 0.35, 0], 'DisplayName', 'GPU ($k_q = 4$)');
% fill([n_states, fliplr(n_states)], [GPU_min, fliplr(GPU_max)], 'g', 'FaceAlpha', 0.15, 'HandleVisibility', 'off');

cpu1 = plot(n_states, CPU_avg, '-o', 'MarkerSize', 14, 'LineWidth', 3, 'Color', 'r', ...
    'MarkerFaceColor', '[1, 0.75, 0.8]', 'MarkerEdgeColor', [0.35, 0, 0], 'DisplayName', 'CPU ($k_q = 4$)');
% fill([n_states, fliplr(n_states)], [CPU_min, fliplr(CPU_max)], 'r', 'FaceAlpha', 0.15, 'HandleVisibility', 'off');


%% WAM GH Degree = 5
% Using the same discretization points
n_states = [25, 50, 100, 200, 300, 500, 750];

% GPU times (in ms)
GPU_avg = [6.36667, 10.6333, 21.0333, 36.2333, 57.5667, 89.2667, 136];
GPU_min = [5, 9, 18, 32, 52, 84, 131];
GPU_max = [8, 12, 25, 42, 72, 99, 143];

% CPU times (in ms)
CPU_avg = [93.4, 178.45, 343.15, 717.4, 1085.5, 1907.8, 3088.4];
CPU_min = [82, 170, 333, 702, 1066, 1873, 3004];
CPU_max = [103, 190, 356, 737, 1117, 1957, 3144];

ratio = (CPU_avg-GPU_avg)./CPU_avg

gpu2 = plot(n_states, GPU_avg, '--x', 'MarkerSize', 16, 'LineWidth', 3, 'Color', 'g', ...
    'MarkerEdgeColor', [0, 0.35, 0], 'DisplayName', 'GPU ($k_q = 10$)');
% fill([n_states, fliplr(n_states)], [GPU_min, fliplr(GPU_max)], 'g', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

cpu2 = plot(n_states, CPU_avg, '--x', 'MarkerSize', 16, 'LineWidth', 3, 'Color', 'r', ...
    'MarkerEdgeColor', [0.35, 0, 0], 'DisplayName', 'CPU ($k_q = 10$)');
% fill([n_states, fliplr(n_states)], [CPU_min, fliplr(CPU_max)], 'r', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');


%% WAM GH Degree = 6
n_states = [25, 50, 100, 200, 300, 500, 750];

% GPU times (in ms)
GPU_avg = [14.9, 21.4333, 49.7667, 78.6667, 117.233, 203.533, 306.167];
GPU_min = [12, 20, 41, 76, 112, 185, 284];
GPU_max = [18, 25, 56, 86, 122, 252, 384];

% CPU times (in ms)
CPU_avg = [294.65, 567.3, 1080.85, 2155.8, 3261.35, 5490.65, 8527.6];
CPU_min = [267, 541, 1048, 2108, 3164, 5343, 8383];
CPU_max = [327, 648, 1143, 2213, 3327, 5735, 8699];

ratio = (CPU_avg-GPU_avg)./CPU_avg

% Uncomment the following lines to plot the GH Degree = 20 curves
gpu3 = plot(n_states, GPU_avg, '-.d', 'MarkerSize', 14, 'LineWidth', 3, 'Color', 'g', ...
    'MarkerFaceColor', [0.5, 1, 0.5], 'MarkerEdgeColor', [0, 0.35, 0], 'DisplayName', 'GPU ($k_q = 20$)');
% fill([n_states, fliplr(n_states)], [GPU_min, fliplr(GPU_max)], 'g', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

cpu3 = plot(n_states, CPU_avg, '-.d', 'MarkerSize', 14, 'LineWidth', 3, 'Color', 'r', ...
    'MarkerFaceColor', [1, 0.75, 0.8], 'MarkerEdgeColor', [0.35, 0, 0], 'DisplayName', 'CPU ($k_q = 20$)');
% fill([n_states, fliplr(n_states)], [CPU_min, fliplr(CPU_max)], 'r', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

xlabel("Time Discretizations")
ylabel("Cost Evaluation Time (ms)")
legend([gpu1 gpu2 gpu3 cpu1 cpu2 cpu3], 'Location', 'northwest');
% legend([gpu1 gpu2 gpu3 cpu1 cpu2], 'Location', 'northwest');
grid minor

set(gcf, 'Toolbar', 'none')
% exportgraphics(gcf, 'WAM_1_updated.pdf', 'ContentType', 'vector', 'BackgroundColor', 'none');
